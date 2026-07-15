// =============================================================================
// SLAMForge ROS2 Node — real-time monocular SLAM for ROS2
// =============================================================================
//
// Subscribes to sensor_msgs/Image, runs the SLAMForge tracking pipeline,
// and publishes:
//   - geometry_msgs/PoseStamped   (camera pose)
//   - sensor_msgs/PointCloud2     (sparse map points)
//   - visualization_msgs/Marker   (keyframe frustums)
//   - TF: odom → camera_link
//
// Build: -DSLAMFORGE_BUILD_ROS2=ON (requires ROS2 Humble/Iron + cv_bridge + tf2_ros)
// =============================================================================

#ifdef SLAMFORGE_HAS_ROS2

#include <cv_bridge/cv_bridge.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <visualization_msgs/msg/marker.hpp>

#include <tf2_ros/transform_broadcaster.h>

#include <chrono>
#include <memory>
#include <string>

#include "slamforge/slamforge.h"

namespace slamforge_ros {

using namespace std::chrono_literals;

/// @brief ROS2 node wrapping the SLAMForge monocular SLAM tracker.
class SLAMForgeNode : public rclcpp::Node {
public:
    SLAMForgeNode() : Node("slamforge_slam") {
        // ── Parameters ──────────────────────────────────────────────────
        this->declare_parameter("config_path", "config/default.yaml");
        this->declare_parameter("output_trajectory", "/tmp/slamforge_traj.txt");
        this->declare_parameter("publish_cloud", true);
        this->declare_parameter("publish_markers", true);
        this->declare_parameter("publish_tf", true);
        this->declare_parameter("camera_frame", "camera_link");
        this->declare_parameter("odom_frame", "odom");

        std::string config_path = this->get_parameter("config_path").as_string();
        output_trajectory_ = this->get_parameter("output_trajectory").as_string();
        publish_cloud_ = this->get_parameter("publish_cloud").as_bool();
        publish_markers_ = this->get_parameter("publish_markers").as_bool();
        publish_tf_ = this->get_parameter("publish_tf").as_bool();
        camera_frame_ = this->get_parameter("camera_frame").as_string();
        odom_frame_ = this->get_parameter("odom_frame").as_string();

        // ── Load configuration ──────────────────────────────────────────
        auto cfg_opt = slamforge::SystemConfig::LoadFromYAML(config_path);
        if (!cfg_opt) {
            RCLCPP_ERROR(this->get_logger(), "Failed to load config from: %s", config_path.c_str());
            rclcpp::shutdown();
            return;
        }
        auto cfg = *cfg_opt;

        // ── Build camera model ──────────────────────────────────────────
        slamforge::Camera::CameraParams cam_params{
            cfg.camera.fx, cfg.camera.fy,    cfg.camera.cx,    cfg.camera.cy,
            cfg.camera.k1, cfg.camera.k2,    cfg.camera.p1,    cfg.camera.p2,
            cfg.camera.k3, cfg.camera.width, cfg.camera.height};
        camera_ = slamforge::Camera::FromParams(cam_params);

        // ── Create tracker ──────────────────────────────────────────────
        tracker_ = std::make_unique<slamforge::tracking::Tracker>(camera_, cfg.tracking, cfg.orb);

        // ── Create local mapper ─────────────────────────────────────────
        slamforge::features::OrbExtractor::Options mapper_opts;
        mapper_opts.num_features = cfg.orb.num_features;
        mapper_opts.scale_factor = static_cast<double>(cfg.orb.scale_factor);
        mapper_opts.num_levels = cfg.orb.num_levels;
        mapper_opts.ini_threshold = cfg.orb.ini_threshold;
        mapper_opts.min_threshold = cfg.orb.min_threshold;
        mapper_opts.patch_size = cfg.orb.patch_size;
        mapper_extractor_ = std::make_unique<slamforge::features::OrbExtractor>(mapper_opts);

        local_mapper_ = std::make_unique<slamforge::mapping::LocalMapper>(
            tracker_->GetMap(), camera_, cfg.mapping, *mapper_extractor_);
        tracker_->SetLocalMapper(local_mapper_.get());

        // ── Create loop closer (optional) ───────────────────────────────
        if (cfg.loop_closing.enabled) {
            loop_closing_ = std::make_unique<slamforge::loop_closing::LoopClosing>(
                tracker_->GetMap(), camera_, cfg.loop_closing);

            if (!cfg.loop_closing.vocab_path.empty()) {
                if (!loop_closing_->LoadVocabulary(cfg.loop_closing.vocab_path)) {
                    RCLCPP_ERROR(this->get_logger(), "Failed to load loop-closing vocabulary: %s",
                                 cfg.loop_closing.vocab_path.c_str());
                    loop_closing_.reset();
                    rclcpp::shutdown();
                    return;
                }
            } else {
                RCLCPP_WARN(this->get_logger(),
                            "Loop closing enabled without vocab_path; using descriptor fallback");
            }

            tracker_->SetLoopClosing(loop_closing_.get());
        }
        local_mapper_->Start();
        if (loop_closing_) {
            loop_closing_->Start();
        }

        // ── Subscribers ─────────────────────────────────────────────────
        image_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
            "image", rclcpp::SensorDataQoS(),
            std::bind(&SLAMForgeNode::ImageCallback, this, std::placeholders::_1));

        // ── Publishers ──────────────────────────────────────────────────
        pose_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("~/pose", 10);
        if (publish_cloud_) {
            cloud_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("~/map_cloud", 10);
        }
        if (publish_markers_) {
            kf_marker_pub_ =
                this->create_publisher<visualization_msgs::msg::Marker>("~/keyframes", 10);
        }

        // ── TF broadcaster ──────────────────────────────────────────────
        if (publish_tf_) {
            tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
        }

        // ── Status timer ────────────────────────────────────────────────
        status_timer_ = this->create_wall_timer(5s, std::bind(&SLAMForgeNode::PublishStatus, this));

        // Open output trajectory file
        traj_file_.open(output_trajectory_);
        if (traj_file_.is_open()) {
            RCLCPP_INFO(this->get_logger(), "Writing trajectory to: %s",
                        output_trajectory_.c_str());
        }

        RCLCPP_INFO(this->get_logger(), "SLAMForge ROS2 node started");
        RCLCPP_INFO(this->get_logger(), "  Camera: %dx%d fx=%.2f", cfg.camera.width,
                    cfg.camera.height, cfg.camera.fx);
    }

    ~SLAMForgeNode() override {
        if (loop_closing_) {
            loop_closing_->Stop();
            if (tracker_) {
                tracker_->SetLoopClosing(nullptr);
            }
        }
        if (local_mapper_) {
            local_mapper_->Stop();
        }
        if (traj_file_.is_open()) {
            traj_file_.close();
        }
    }

private:
    /// @brief Process incoming image messages.
    void ImageCallback(const sensor_msgs::msg::Image::SharedPtr msg) {
        // Convert ROS image to OpenCV
        cv_bridge::CvImagePtr cv_ptr;
        try {
            cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::MONO8);
        } catch (const cv_bridge::Exception& e) {
            RCLCPP_ERROR(this->get_logger(), "cv_bridge error: %s", e.what());
            return;
        }

        // Get timestamp
        double timestamp = msg->header.stamp.sec + msg->header.stamp.nanosec * 1e-9;

        // Track
        auto pose_opt = tracker_->Track(cv_ptr->image, timestamp);

        if (pose_opt) {
            // Publish pose
            PublishPose(*pose_opt, msg->header.stamp, msg->header.frame_id);

            // Publish TF
            if (publish_tf_) {
                PublishTF(*pose_opt, msg->header.stamp);
            }

            // Write trajectory
            WriteTrajectory(*pose_opt, timestamp);

            // Periodically publish map cloud and keyframe markers
            frame_count_++;
            if (publish_cloud_ && frame_count_ % 10 == 0) {
                PublishMapCloud(msg->header.stamp);
            }
            if (publish_markers_ && frame_count_ % 30 == 0) {
                PublishKeyFrameMarkers(msg->header.stamp);
            }
        } else {
            lost_count_++;
        }
    }

    /// @brief Publish camera pose as PoseStamped.
    void PublishPose(const slamforge::SE3& Tcw, const rclcpp::Time& stamp,
                     const std::string& frame_id) {
        auto msg = geometry_msgs::msg::PoseStamped();
        msg.header.stamp = stamp;
        msg.header.frame_id = odom_frame_;

        // Tcw = world-to-camera → invert to get camera-to-world
        slamforge::SE3 Twc = Tcw.inverse();
        slamforge::Pose p = slamforge::Pose::FromSE3(Twc);

        msg.pose.position.x = p.position.x();
        msg.pose.position.y = p.position.y();
        msg.pose.position.z = p.position.z();
        msg.pose.orientation.x = p.orientation.x();
        msg.pose.orientation.y = p.orientation.y();
        msg.pose.orientation.z = p.orientation.z();
        msg.pose.orientation.w = p.orientation.w();

        pose_pub_->publish(msg);
    }

    /// @brief Broadcast TF from odom → camera.
    void PublishTF(const slamforge::SE3& Tcw, const rclcpp::Time& stamp) {
        slamforge::SE3 Twc = Tcw.inverse();
        slamforge::Pose p = slamforge::Pose::FromSE3(Twc);

        auto transform = geometry_msgs::msg::TransformStamped();
        transform.header.stamp = stamp;
        transform.header.frame_id = odom_frame_;
        transform.child_frame_id = camera_frame_;

        transform.transform.translation.x = p.position.x();
        transform.transform.translation.y = p.position.y();
        transform.transform.translation.z = p.position.z();
        transform.transform.rotation.x = p.orientation.x();
        transform.transform.rotation.y = p.orientation.y();
        transform.transform.rotation.z = p.orientation.z();
        transform.transform.rotation.w = p.orientation.w();

        tf_broadcaster_->sendTransform(transform);
    }

    /// @brief Publish sparse map as PointCloud2.
    void PublishMapCloud(const rclcpp::Time& stamp) {
        auto msg = sensor_msgs::msg::PointCloud2();
        msg.header.stamp = stamp;
        msg.header.frame_id = odom_frame_;

        auto points = tracker_->GetMap().GetAllMapPoints();
        msg.width = points.size();
        msg.height = 1;
        msg.is_dense = false;

        sensor_msgs::PointCloud2Modifier modifier(msg);
        modifier.setPointCloud2Fields(3, "x", 1, sensor_msgs::msg::PointField::FLOAT32, "y", 1,
                                      sensor_msgs::msg::PointField::FLOAT32, "z", 1,
                                      sensor_msgs::msg::PointField::FLOAT32);
        modifier.resize(points.size());

        sensor_msgs::PointCloud2Iterator<float> iter_x(msg, "x");
        sensor_msgs::PointCloud2Iterator<float> iter_y(msg, "y");
        sensor_msgs::PointCloud2Iterator<float> iter_z(msg, "z");

        for (const auto& mp : points) {
            auto pos = mp->Position();
            *iter_x = static_cast<float>(pos.x());
            *iter_y = static_cast<float>(pos.y());
            *iter_z = static_cast<float>(pos.z());
            ++iter_x;
            ++iter_y;
            ++iter_z;
        }

        cloud_pub_->publish(msg);
    }

    /// @brief Publish keyframe camera frustums as Marker.
    void PublishKeyFrameMarkers(const rclcpp::Time& stamp) {
        auto msg = visualization_msgs::msg::Marker();
        msg.header.stamp = stamp;
        msg.header.frame_id = odom_frame_;
        msg.ns = "keyframes";
        msg.id = 0;
        msg.type = visualization_msgs::msg::Marker::LINE_LIST;
        msg.action = visualization_msgs::msg::Marker::ADD;
        msg.scale.x = 0.02;  // line width
        msg.color.r = 0.0;
        msg.color.g = 1.0;
        msg.color.b = 0.0;
        msg.color.a = 0.7;

        auto keyframes = tracker_->GetMap().GetAllKeyFrames();
        msg.points.reserve(keyframes.size() * 8);

        for (const auto& kf : keyframes) {
            // Camera center in world frame
            slamforge::Vec3 center = kf->CameraCenter();
            slamforge::SE3 Twc = kf->Pose().inverse();
            slamforge::Mat3 R = Twc.rotation();

            // Four corners of a virtual frustum at 1m distance
            double f = 0.5;  // half-size of frustum face
            slamforge::Vec3 corners[4] = {
                Twc * slamforge::Vec3(f, f, 1.0),
                Twc * slamforge::Vec3(-f, f, 1.0),
                Twc * slamforge::Vec3(-f, -f, 1.0),
                Twc * slamforge::Vec3(f, -f, 1.0),
            };

            // Lines from center to each corner + outline
            for (int i = 0; i < 4; i++) {
                auto add_point = [&](const slamforge::Vec3& p) {
                    geometry_msgs::msg::Point pt;
                    pt.x = p.x();
                    pt.y = p.y();
                    pt.z = p.z();
                    msg.points.push_back(pt);
                };
                add_point(center);
                add_point(corners[i]);
                add_point(corners[i]);
                add_point(corners[(i + 1) % 4]);
            }
        }

        kf_marker_pub_->publish(msg);
    }

    /// @brief Write a pose to the TUM trajectory file.
    void WriteTrajectory(const slamforge::SE3& Tcw, double timestamp) {
        if (!traj_file_.is_open())
            return;

        slamforge::SE3 Twc = Tcw.inverse();
        slamforge::Pose p = slamforge::Pose::FromSE3(Twc);

        traj_file_ << std::fixed << std::setprecision(9) << timestamp << " " << p.position.x()
                   << " " << p.position.y() << " " << p.position.z() << " " << p.orientation.x()
                   << " " << p.orientation.y() << " " << p.orientation.z() << " "
                   << p.orientation.w() << "\n";
    }

    /// @brief Periodic status logging.
    void PublishStatus() {
        auto state = tracker_->State();
        std::string state_str;
        switch (state) {
            case slamforge::tracking::TrackingState::NOT_INITIALIZED:
                state_str = "NOT_INIT";
                break;
            case slamforge::tracking::TrackingState::INITIALIZING:
                state_str = "INITIALIZING";
                break;
            case slamforge::tracking::TrackingState::OK:
                state_str = "OK";
                break;
            case slamforge::tracking::TrackingState::LOST:
                state_str = "LOST";
                break;
        }

        RCLCPP_INFO(this->get_logger(),
                    "State=%s | Frames=%d | Lost=%d | Tracked=%d | KFs=%zu | MPs=%zu",
                    state_str.c_str(), frame_count_, lost_count_, tracker_->NumTrackedPoints(),
                    tracker_->GetMap().KeyFrameCount(), tracker_->GetMap().MapPointCount());
    }

    // ── SLAMForge components ───────────────────────────────────────────────
    slamforge::Camera camera_;
    std::unique_ptr<slamforge::tracking::Tracker> tracker_;
    std::unique_ptr<slamforge::features::OrbExtractor> mapper_extractor_;
    std::unique_ptr<slamforge::mapping::LocalMapper> local_mapper_;
    std::unique_ptr<slamforge::loop_closing::LoopClosing> loop_closing_;

    // ── ROS2 communication ──────────────────────────────────────────────
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_pub_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_pub_;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr kf_marker_pub_;
    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
    rclcpp::TimerBase::SharedPtr status_timer_;

    // ── Parameters ──────────────────────────────────────────────────────
    std::string output_trajectory_;
    bool publish_cloud_;
    bool publish_markers_;
    bool publish_tf_;
    std::string camera_frame_;
    std::string odom_frame_;

    // ── Bookkeeping ─────────────────────────────────────────────────────
    int frame_count_ = 0;
    int lost_count_ = 0;
    std::ofstream traj_file_;
};

}  // namespace slamforge_ros

// ── Main ────────────────────────────────────────────────────────────────────

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<slamforge_ros::SLAMForgeNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}

#else  // SLAMFORGE_HAS_ROS2

#include <cstdio>

int main() {
    std::fprintf(stderr, "Error: SLAMForge was built without ROS2 support.\n");
    std::fprintf(stderr, "Rebuild with: -DSLAMFORGE_BUILD_ROS2=ON\n");
    return 1;
}

#endif  // SLAMFORGE_HAS_ROS2
