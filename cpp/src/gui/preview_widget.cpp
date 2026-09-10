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
    has_scene_ = true;
    needs_fit_ = true;
    update();
}

void PreviewWidget::clear_scene()
{
    scene_ = {};
    has_scene_ = false;
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
    view_ = preview::View2D::fit(scene_.bounds(), width(), height(), 20.0);
    needs_fit_ = false;
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
        view_.origin_x += delta.x();
        view_.origin_y += delta.y();
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
    const auto world = view_.from_canvas({position.x(), position.y()});
    const double factor = std::pow(1.15, static_cast<double>(event->angleDelta().y()) / 120.0);
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

void PreviewWidget::draw_scale_bar(QPainter& painter) const
{
    if (!has_scene_) return;
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
