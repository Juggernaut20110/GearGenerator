#include "gui/preview_widget.hpp"

#include <QColor>
#include <QMouseEvent>
#include <QPainter>
#include <QResizeEvent>
#include <QWheelEvent>

#include <algorithm>
#include <cmath>

namespace geargen::gui {

PreviewWidget::PreviewWidget(QWidget* parent)
    : QWidget(parent)
{
    setMinimumSize(420, 300);
    setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Expanding);
    setMouseTracking(true);
    setAutoFillBackground(true);
    QPalette palette = this->palette();
    palette.setColor(QPalette::Base, Qt::white);
    setPalette(palette);
}

void PreviewWidget::set_scene(const preview::Scene2D& scene)
{
    scene_ = scene;
    scene3d_ = {};
    has_scene_ = true;
    is_3d_ = false;
    needs_fit_ = true;
    update();
}

void PreviewWidget::set_scene3d(const preview::Scene3D& scene)
{
    scene3d_ = scene;
    scene_ = {};
    has_scene_ = true;
    is_3d_ = true;
    needs_fit_ = true;
    update();
}

void PreviewWidget::clear_scene()
{
    scene_ = {};
    scene3d_ = {};
    has_scene_ = false;
    is_3d_ = false;
    needs_fit_ = true;
    update();
}

void PreviewWidget::fit_view()
{
    needs_fit_ = true;
    ensure_fit();
    update();
}

void PreviewWidget::ensure_fit()
{
    if (!has_scene_ || !needs_fit_ || width() <= 0 || height() <= 0) return;
    if (is_3d_) camera_.fit_scene(scene3d_, width(), height(), 20.0);
    else view_ = preview::View2D::fit(scene_.bounds(), width(), height(), 20.0);
    needs_fit_ = false;
}

void PreviewWidget::set_camera_view(const std::string& name)
{
    if (!has_scene_ || !is_3d_) return;
    camera_.set_view(name);
    needs_fit_ = true;
    ensure_fit();
    update();
}

void PreviewWidget::paintEvent(QPaintEvent*)
{
    ensure_fit();
    QPainter painter(this);
    painter.setRenderHint(QPainter::Antialiasing, true);
    painter.fillRect(rect(), Qt::white);
    if (!has_scene_) {
        painter.setPen(QColor(QStringLiteral("#777777")));
        painter.drawText(rect(), Qt::AlignCenter,
                         QStringLiteral("Enter valid parameters to preview geometry."));
        return;
    }

    if (is_3d_) {
        for (const auto& line : scene3d_.polylines) {
            if (line.points.size() < 2U) continue;
            const auto points = preview::drawn_points(line);
            if (points.size() < 2U) continue;
            QPolygonF polygon;
            polygon.reserve(static_cast<int>(points.size()));
            bool finite = true;
            for (const auto point : points) {
                if (!std::isfinite(point.x) || !std::isfinite(point.y) ||
                    !std::isfinite(point.z)) {
                    finite = false;
                    break;
                }
                const auto canvas = camera_.project(
                    point, static_cast<double>(width()), static_cast<double>(height()));
                polygon.append(QPointF(canvas.x, canvas.y));
            }
            if (!finite) continue;
            const auto& style = preview::style_for_3d(line.style);
            QPen pen(QColor(QString::fromStdString(style.colour)));
            pen.setWidthF(style.width);
            if (!style.dash.empty()) {
                QVector<qreal> pattern;
                pattern.reserve(static_cast<int>(style.dash.size()));
                for (const int dash : style.dash) pattern.push_back(static_cast<qreal>(dash));
                pen.setDashPattern(pattern);
            }
            painter.setPen(pen);
            painter.drawPolyline(polygon);
        }
        draw_legend3d(painter);
        return;
    }

    for (const auto& line : scene_.polylines) {
        if (line.points.size() < 2U) continue;
        const auto points = preview::drawn_points(line);
        QPolygonF polygon;
        polygon.reserve(static_cast<int>(points.size()));
        for (const auto point : points) {
            const auto canvas = view_.to_canvas(point);
            polygon.append(QPointF(canvas.x, canvas.y));
        }
        const auto& style = preview::style_for(line.style);
        QPen pen(QColor(QString::fromStdString(style.colour)));
        pen.setWidthF(style.width);
        if (!style.dash.empty()) {
            QVector<qreal> pattern;
            pattern.reserve(static_cast<int>(style.dash.size()));
            for (const int dash : style.dash) pattern.push_back(static_cast<qreal>(dash));
            pen.setDashPattern(pattern);
        }
        painter.setPen(pen);
        painter.drawPolyline(polygon);
    }
    draw_legend(painter);
    draw_scale_bar(painter);
}

void PreviewWidget::resizeEvent(QResizeEvent* event)
{
    QWidget::resizeEvent(event);
    if (has_scene_) {
        needs_fit_ = true;
        update();
    }
}

void PreviewWidget::mousePressEvent(QMouseEvent* event)
{
    if (event->button() == Qt::LeftButton || event->button() == Qt::MiddleButton ||
        event->button() == Qt::RightButton) {
        dragging_ = true;
        drag_button_ = event->button();
        last_drag_position_ = event->position().toPoint();
        setCursor(Qt::ClosedHandCursor);
        event->accept();
        return;
    }
    QWidget::mousePressEvent(event);
}

void PreviewWidget::mouseMoveEvent(QMouseEvent* event)
{
    if (dragging_) {
        const QPoint current = event->position().toPoint();
        const QPoint delta = current - last_drag_position_;
        if (is_3d_) {
            if (drag_button_ == Qt::LeftButton) camera_.orbit(delta.x(), delta.y());
            else camera_.pan(delta.x(), delta.y());
        } else {
            view_.origin_x += delta.x();
            view_.origin_y += delta.y();
        }
        last_drag_position_ = current;
        update();
        event->accept();
        return;
    }
    QWidget::mouseMoveEvent(event);
}

void PreviewWidget::mouseReleaseEvent(QMouseEvent* event)
{
    dragging_ = false;
    drag_button_ = Qt::NoButton;
    unsetCursor();
    QWidget::mouseReleaseEvent(event);
}

void PreviewWidget::mouseDoubleClickEvent(QMouseEvent* event)
{
    if (event->button() == Qt::LeftButton) {
        fit_view();
        event->accept();
        return;
    }
    QWidget::mouseDoubleClickEvent(event);
}

void PreviewWidget::wheelEvent(QWheelEvent* event)
{
    ensure_fit();
    if (!has_scene_ || event->angleDelta().y() == 0) return;
    const QPointF position = event->position();
    const double factor = std::pow(
        1.15, static_cast<double>(event->angleDelta().y()) / 120.0);
    if (is_3d_) {
        camera_.zoom_at(factor, position.x(), position.y(),
                        static_cast<double>(width()), static_cast<double>(height()));
        update();
        event->accept();
        return;
    }
    const auto world = view_.from_canvas({position.x(), position.y()});
    const double new_scale = std::clamp(view_.scale * factor, 1e-6, 1e9);
    view_.scale = new_scale;
    view_.origin_x = position.x() - world.x * new_scale;
    view_.origin_y = position.y() + world.y * new_scale;
    update();
    event->accept();
}

void PreviewWidget::draw_legend(QPainter& painter) const
{
    double y = 18.0;
    for (const auto& [style_name, label] : scene_.legend) {
        const auto& style = preview::style_for(style_name);
        QPen pen(QColor(QString::fromStdString(style.colour)));
        pen.setWidthF(std::max(style.width, 1.6));
        if (!style.dash.empty()) {
            QVector<qreal> pattern;
            for (const int dash : style.dash) pattern.push_back(static_cast<qreal>(dash));
            pen.setDashPattern(pattern);
        }
        painter.setPen(pen);
        painter.drawLine(QPointF(12.0, y), QPointF(34.0, y));
        painter.setPen(QColor(QStringLiteral("#444444")));
        painter.drawText(QPointF(40.0, y + 4.0), QString::fromStdString(label));
        y += 14.0;
    }
}

void PreviewWidget::draw_legend3d(QPainter& painter) const
{
    double y = 18.0;
    for (const auto& [style_name, label] : scene3d_.legend) {
        const auto& style = preview::style_for_3d(style_name);
        QPen pen(QColor(QString::fromStdString(style.colour)));
        pen.setWidthF(std::max(style.width, 1.6));
        if (!style.dash.empty()) {
            QVector<qreal> pattern;
            pattern.reserve(static_cast<int>(style.dash.size()));
            for (const int dash : style.dash) pattern.push_back(static_cast<qreal>(dash));
            pen.setDashPattern(pattern);
        }
        painter.setPen(pen);
        painter.drawLine(QPointF(12.0, y), QPointF(34.0, y));
        painter.setPen(QColor(QStringLiteral("#444444")));
        painter.drawText(QPointF(40.0, y + 4.0), QString::fromStdString(label));
        y += 14.0;
    }
}

void PreviewWidget::draw_scale_bar(QPainter& painter) const
{
    if (!has_scene_ || is_3d_) return;
    const double length_mm = preview::nice_length(view_.scale);
    const double length_px = length_mm * view_.scale;
    const double x1 = width() - 16.0;
    const double x0 = x1 - length_px;
    const double y = height() - 16.0;
    const auto style = preview::style_for(std::string("scale"));
    QPen pen(QColor(QString::fromStdString(style.colour)));
    pen.setWidthF(style.width);
    painter.setPen(pen);
    painter.drawLine(QPointF(x0, y), QPointF(x1, y));
    painter.drawLine(QPointF(x0, y - 4.0), QPointF(x0, y + 4.0));
    painter.drawLine(QPointF(x1, y - 4.0), QPointF(x1, y + 4.0));
    painter.drawText(QPointF(0.5 * (x0 + x1), y - 8.0),
                     QStringLiteral("%1 mm").arg(length_mm, 0, 'g'));
}

} // namespace geargen::gui
