#pragma once

#include <QColor>
#include <QPoint>
#include <QString>
#include <QVector3D>
#include <QVector>
#include <QWidget>

class QMouseEvent;
class QPaintEvent;
class QWheelEvent;

namespace slamforge::desktop {

/// Lightweight interactive viewer for a sparse PLY map and camera trajectory.
///
/// The renderer deliberately uses QPainter instead of a separate 3D runtime so
/// the first desktop package remains portable across software, integrated, and
/// discrete GPU configurations. Large maps are uniformly sampled on load.
class MapView final : public QWidget {
public:
    explicit MapView(QWidget* parent = nullptr);

    /// Load an ASCII PLY point cloud and an auto-detected SLAMForge trajectory.
    /// Returns true when at least one map point or pose can be displayed.
    bool LoadResult(const QString& map_path, const QString& trajectory_path,
                    QString* error = nullptr);
    void Clear();

    [[nodiscard]] qsizetype MapPointCount() const { return map_points_.size(); }
    [[nodiscard]] qsizetype PoseCount() const { return trajectory_.size(); }

protected:
    void paintEvent(QPaintEvent* event) override;
    void mousePressEvent(QMouseEvent* event) override;
    void mouseMoveEvent(QMouseEvent* event) override;
    void mouseDoubleClickEvent(QMouseEvent* event) override;
    void wheelEvent(QWheelEvent* event) override;

private:
    bool LoadPly(const QString& path, QString* error);
    bool LoadTrajectory(const QString& path, QString* error);
    void FitData();
    void ResetView();
    [[nodiscard]] QPointF Project(const QVector3D& point, float* depth = nullptr) const;

    QVector<QVector3D> map_points_;
    QVector<QColor> map_colors_;
    QVector<QVector3D> trajectory_;
    QVector3D center_;
    float radius_ = 1.0F;
    float yaw_ = 0.65F;
    float pitch_ = -0.4F;
    float zoom_ = 1.0F;
    QPoint last_mouse_position_;
    QString message_;
};

}  // namespace slamforge::desktop
