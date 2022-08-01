//
// Created by pbustos on 28/07/22.
//

#ifndef LOCAL_GRID_QGRAPHICSCELLITEM_H
#define LOCAL_GRID_QGRAPHICSCELLITEM_H

#include <QPainter>
#include <QGraphicsItem>
#include <math.h>

class QGraphicsCellItem : public QGraphicsItem
{
    public:
        QGraphicsCellItem(float ang_, float rad_, float ang_step_, float rad_step_);
        QRectF boundingRect() const;

        // overriding paint()
        void paint(QPainter * painter, const QStyleOptionGraphicsItem * option, QWidget * widget);
        void setFreeColor();
        void setOccupiedColor(int level);

    protected:
        // overriding mouse events
        void mousePressEvent(QGraphicsSceneMouseEvent *event);
        void mouseReleaseEvent(QGraphicsSceneMouseEvent *event);

    private:
        float ang, rad, ang_step, rad_step;
        QString free_color = "lightgreen";
        QString occupied_color_0 = "red";
        QString occupied_color_1 = "orange";
        QString occupied_color_2 = "yellow";
        QString occupied_color_3 = "lightgrey";

        QColor free_pen = QColor(free_color);
        QColor occupied_pen = QColor(occupied_color_1);
        QBrush free_brush = QBrush(free_pen);
        std::vector<QBrush> occupied_brushes = {QBrush(QColor(occupied_color_0)), QBrush(QColor(occupied_color_1)), QBrush(QColor(occupied_color_2)), QBrush(QColor(occupied_color_3))};
        QBrush current_brush = free_brush;

        QPointF A,B,C,D;
        QPolygonF poly;
        inline float radians_to_degrees(float a) const { if(a>=0) return a*180.f/M_PI; else return a*180/M_PI+360;};
        float opacity = 0.1;
};

#endif //LOCAL_GRID_QGRAPHICSCELLITEM_H
