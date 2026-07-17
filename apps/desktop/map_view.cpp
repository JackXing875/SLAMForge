#include "map_view.h"

#include <QFile>
#include <QFileInfo>
#include <QFontMetrics>
#include <QImage>
#include <QMouseEvent>
#include <QPaintEvent>
#include <QPainter>
#include <QPainterPath>
#include <QRegularExpression>
#include <QTextStream>
#include <QWheelEvent>
#include <algorithm>
#include <cmath>
#include <limits>
#include <vector>

namespace slamforge::desktop {
namespace {

constexpr qsizetype kMaximumDisplayedPoints = 250000;
constexpr qsizetype kMaximumDisplayedPoses = 50000;

bool ParseVector(const QStringList& fields, qsizetype x_index, qsizetype y_index, qsizetype z_index,
                 QVector3D* output) {
    if (output == nullptr || fields.size() <= std::max({x_index, y_index, z_index})) {
        return false;
    }
    bool x_ok = false;
    bool y_ok = false;
    bool z_ok = false;
    const float x = fields[x_index].toFloat(&x_ok);
    const float y = fields[y_index].toFloat(&y_ok);
    const float z = fields[z_index].toFloat(&z_ok);
    if (!x_ok || !y_ok || !z_ok || !std::isfinite(x) || !std::isfinite(y) || !std::isfinite(z)) {
        return false;
    }
    *output = QVector3D(x, y, z);
    return true;
}

}  // namespace

MapView::MapView(QWidget* parent) : QWidget(parent) {
    setMinimumSize(520, 320);
    setMouseTracking(true);
    setFocusPolicy(Qt::StrongFocus);
    Clear();
}

bool MapView::LoadResult(const QString& map_path, const QString& trajectory_path, QString* error) {
    map_points_.clear();
    map_colors_.clear();
    trajectory_.clear();

    QStringList errors;
    QString map_error;
    if (!LoadPly(map_path, &map_error) && !map_error.isEmpty()) {
        errors.push_back(map_error);
    }
    QString trajectory_error;
    if (!LoadTrajectory(trajectory_path, &trajectory_error) && !trajectory_error.isEmpty()) {
        errors.push_back(trajectory_error);
    }

    if (map_points_.isEmpty() && trajectory_.isEmpty()) {
        message_ = tr("No valid map points or camera poses were produced.");
        if (error != nullptr) {
            *error = errors.isEmpty() ? message_ : errors.join(QLatin1Char('\n'));
        }
        update();
        return false;
    }

    FitData();
    message_.clear();
    if (error != nullptr) {
        *error = errors.join(QLatin1Char('\n'));
    }
    update();
    return true;
}

void MapView::Clear() {
    map_points_.clear();
    map_colors_.clear();
    trajectory_.clear();
    center_ = QVector3D();
    radius_ = 1.0F;
    message_ = tr("Run an analysis to display the colored surface map and camera trajectory.");
    ResetView();
}

bool MapView::LoadPly(const QString& path, QString* error) {
    QFile file(path);
    if (!file.open(QIODevice::ReadOnly | QIODevice::Text)) {
        if (error != nullptr) {
            *error = tr("Cannot open map file: %1").arg(QFileInfo(path).fileName());
        }
        return false;
    }

    QTextStream stream(&file);
    qint64 vertex_count = -1;
    bool ascii_format = false;
    bool header_complete = false;
    while (!stream.atEnd()) {
        const QString line = stream.readLine().trimmed();
        if (line == QStringLiteral("format ascii 1.0")) {
            ascii_format = true;
        } else if (line.startsWith(QStringLiteral("element vertex "))) {
            bool count_ok = false;
            vertex_count = line.sliced(15).toLongLong(&count_ok);
            if (!count_ok || vertex_count < 0) {
                vertex_count = -1;
            }
        } else if (line == QStringLiteral("end_header")) {
            header_complete = true;
            break;
        }
    }

    if (!ascii_format || !header_complete || vertex_count < 0) {
        if (error != nullptr) {
            *error = tr("The map is not a supported ASCII PLY file.");
        }
        return false;
    }

    const qint64 stride =
        std::max<qint64>(1, (vertex_count + kMaximumDisplayedPoints - 1) / kMaximumDisplayedPoints);
    map_points_.reserve(static_cast<qsizetype>(
        std::min<qint64>(vertex_count, static_cast<qint64>(kMaximumDisplayedPoints))));
    map_colors_.reserve(map_points_.capacity());
    static const QRegularExpression whitespace(QStringLiteral(R"(\s+)"));
    for (qint64 index = 0; index < vertex_count && !stream.atEnd(); ++index) {
        const QString line = stream.readLine().trimmed();
        if ((index % stride) != 0) {
            continue;
        }
        const QStringList fields = line.split(whitespace, Qt::SkipEmptyParts);
        QVector3D point;
        if (ParseVector(fields, 0, 1, 2, &point)) {
            map_points_.push_back(point);
            bool red_ok = false;
            bool green_ok = false;
            bool blue_ok = false;
            const int red = fields.size() > 3 ? fields[3].toInt(&red_ok) : 68;
            const int green = fields.size() > 4 ? fields[4].toInt(&green_ok) : 160;
            const int blue = fields.size() > 5 ? fields[5].toInt(&blue_ok) : 220;
            map_colors_.push_back(red_ok && green_ok && blue_ok
                                      ? QColor(std::clamp(red, 0, 255), std::clamp(green, 0, 255),
                                               std::clamp(blue, 0, 255))
                                      : QColor(68, 160, 220));
        }
    }
    return true;
}

bool MapView::LoadTrajectory(const QString& path, QString* error) {
    QFile file(path);
    if (!file.open(QIODevice::ReadOnly | QIODevice::Text)) {
        if (error != nullptr) {
            *error = tr("Cannot open trajectory file: %1").arg(QFileInfo(path).fileName());
        }
        return false;
    }

    QVector<QVector3D> poses;
    QTextStream stream(&file);
    static const QRegularExpression whitespace(QStringLiteral(R"(\s+)"));
    while (!stream.atEnd()) {
        QString line = stream.readLine().trimmed();
        if (line.isEmpty() || line.startsWith(QLatin1Char('#'))) {
            continue;
        }
        line.replace(QLatin1Char(','), QLatin1Char(' '));
        const QStringList fields = line.split(whitespace, Qt::SkipEmptyParts);
        QVector3D pose;
        const bool is_kitti = fields.size() == 12;
        const bool valid =
            is_kitti ? ParseVector(fields, 3, 7, 11, &pose) : ParseVector(fields, 1, 2, 3, &pose);
        if (valid) {
            poses.push_back(pose);
        }
    }

    const qsizetype stride = std::max<qsizetype>(
        1, (poses.size() + kMaximumDisplayedPoses - 1) / kMaximumDisplayedPoses);
    trajectory_.reserve(std::min(poses.size(), kMaximumDisplayedPoses));
    for (qsizetype index = 0; index < poses.size(); index += stride) {
        trajectory_.push_back(poses[index]);
    }
    return true;
}

void MapView::FitData() {
    QVector3D minimum(std::numeric_limits<float>::max(), std::numeric_limits<float>::max(),
                      std::numeric_limits<float>::max());
    QVector3D maximum(std::numeric_limits<float>::lowest(), std::numeric_limits<float>::lowest(),
                      std::numeric_limits<float>::lowest());
    const auto extend_bounds = [&minimum, &maximum](const QVector3D& point) {
        minimum.setX(std::min(minimum.x(), point.x()));
        minimum.setY(std::min(minimum.y(), point.y()));
        minimum.setZ(std::min(minimum.z(), point.z()));
        maximum.setX(std::max(maximum.x(), point.x()));
        maximum.setY(std::max(maximum.y(), point.y()));
        maximum.setZ(std::max(maximum.z(), point.z()));
    };
    for (const QVector3D& point : map_points_) {
        extend_bounds(point);
    }
    for (const QVector3D& pose : trajectory_) {
        extend_bounds(pose);
    }

    center_ = (minimum + maximum) * 0.5F;
    radius_ = std::max(0.0001F, (maximum - minimum).length() * 0.5F);
    ResetView();
}

void MapView::ResetView() {
    yaw_ = 0.65F;
    pitch_ = -0.4F;
    zoom_ = 1.0F;
    update();
}

QPointF MapView::Project(const QVector3D& point, float* depth) const {
    const QVector3D relative = point - center_;
    const float cosine_yaw = std::cos(yaw_);
    const float sine_yaw = std::sin(yaw_);
    const float rotated_x = cosine_yaw * relative.x() + sine_yaw * relative.z();
    const float rotated_z = -sine_yaw * relative.x() + cosine_yaw * relative.z();

    const float cosine_pitch = std::cos(pitch_);
    const float sine_pitch = std::sin(pitch_);
    const float rotated_y = cosine_pitch * relative.y() - sine_pitch * rotated_z;
    const float projected_depth = sine_pitch * relative.y() + cosine_pitch * rotated_z;
    if (depth != nullptr) {
        *depth = projected_depth;
    }

    const float viewport = static_cast<float>(std::min(width(), height()));
    const float scale = viewport * 0.43F * zoom_ / radius_;
    return {static_cast<qreal>(width()) * 0.5 + static_cast<qreal>(rotated_x * scale),
            static_cast<qreal>(height()) * 0.5 - static_cast<qreal>(rotated_y * scale)};
}

void MapView::paintEvent(QPaintEvent* event) {
    Q_UNUSED(event)
    QPainter painter(this);
    painter.fillRect(rect(), QColor(14, 19, 27));

    if (map_points_.isEmpty() && trajectory_.isEmpty()) {
        painter.setPen(QColor(164, 174, 188));
        painter.drawText(rect().adjusted(30, 30, -30, -30), Qt::AlignCenter | Qt::TextWordWrap,
                         message_);
        return;
    }

    // Render the surface through a small software z-buffer. Drawing points in
    // file or color-bucket order lets geometry behind a wall overwrite the
    // visible wall and makes an otherwise dense map look transparent.
    QImage surface(size(), QImage::Format_ARGB32);
    surface.fill(Qt::transparent);
    const int surface_width = surface.width();
    const int surface_height = surface.height();
    std::vector<float> depth_buffer(
        static_cast<std::size_t>(surface_width) * static_cast<std::size_t>(surface_height),
        std::numeric_limits<float>::lowest());
    for (qsizetype index = 0; index < map_points_.size(); ++index) {
        const QColor color = index < map_colors_.size() ? map_colors_[index] : QColor(68, 160, 220);
        float depth = 0.0F;
        const QPointF projected = Project(map_points_[index], &depth);
        const int center_x = qRound(projected.x());
        const int center_y = qRound(projected.y());
        const QRgb pixel = qRgba(color.red(), color.green(), color.blue(), 238);
        for (int offset_y = -1; offset_y <= 1; ++offset_y) {
            const int y = center_y + offset_y;
            if (y < 0 || y >= surface_height) {
                continue;
            }
            auto* row = reinterpret_cast<QRgb*>(surface.scanLine(y));
            for (int offset_x = -1; offset_x <= 1; ++offset_x) {
                const int x = center_x + offset_x;
                if (x < 0 || x >= surface_width) {
                    continue;
                }
                const std::size_t buffer_index =
                    static_cast<std::size_t>(y) * static_cast<std::size_t>(surface_width) +
                    static_cast<std::size_t>(x);
                if (depth > depth_buffer[buffer_index]) {
                    depth_buffer[buffer_index] = depth;
                    row[x] = pixel;
                }
            }
        }
    }
    painter.setRenderHint(QPainter::Antialiasing, false);
    painter.drawImage(QPoint(0, 0), surface);

    if (!trajectory_.isEmpty()) {
        painter.setRenderHint(QPainter::Antialiasing, true);
        QPainterPath path;
        path.moveTo(Project(trajectory_.front()));
        for (qsizetype index = 1; index < trajectory_.size(); ++index) {
            path.lineTo(Project(trajectory_[index]));
        }
        painter.setPen(QPen(QColor(255, 174, 66), 2.2));
        painter.drawPath(path);
        painter.setPen(Qt::NoPen);
        painter.setBrush(QColor(255, 216, 112));
        painter.drawEllipse(Project(trajectory_.back()), 4.5, 4.5);
    }

    painter.setRenderHint(QPainter::Antialiasing, true);
    painter.setPen(QColor(220, 228, 239));
    painter.drawText(QRect(16, 12, width() - 32, 24), Qt::AlignLeft | Qt::AlignVCenter,
                     tr("%1 surface points  •  %2 camera poses  •  relative scale")
                         .arg(map_points_.size())
                         .arg(trajectory_.size()));
    painter.setPen(QColor(132, 145, 163));
    painter.drawText(QRect(16, height() - 34, width() - 32, 22), Qt::AlignRight | Qt::AlignVCenter,
                     tr("Drag to rotate  •  Wheel to zoom  •  Double-click to reset"));
}

void MapView::mousePressEvent(QMouseEvent* event) {
    last_mouse_position_ = event->position().toPoint();
    event->accept();
}

void MapView::mouseMoveEvent(QMouseEvent* event) {
    if ((event->buttons() & Qt::LeftButton) == 0) {
        return;
    }
    const QPoint current = event->position().toPoint();
    const QPoint delta = current - last_mouse_position_;
    last_mouse_position_ = current;
    yaw_ += static_cast<float>(delta.x()) * 0.008F;
    pitch_ = std::clamp(pitch_ + static_cast<float>(delta.y()) * 0.008F, -1.5F, 1.5F);
    update();
    event->accept();
}

void MapView::mouseDoubleClickEvent(QMouseEvent* event) {
    ResetView();
    event->accept();
}

void MapView::wheelEvent(QWheelEvent* event) {
    const float zoom_factor = std::pow(1.0015F, static_cast<float>(event->angleDelta().y()));
    zoom_ = std::clamp(zoom_ * zoom_factor, 0.15F, 20.0F);
    update();
    event->accept();
}

}  // namespace slamforge::desktop
