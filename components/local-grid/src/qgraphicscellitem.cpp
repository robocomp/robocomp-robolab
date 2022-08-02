//
// Created by pbustos on 28/07/22.
//

#include "qgraphicscellitem.h"
#include <math.h>
#include <QtMath>
#include <QDebug>

QGraphicsCellItem::QGraphicsCellItem(float ang_, float rad_, float ang_step_, float rad_step_) :
        ang(ang_), rad(rad_), ang_step(ang_step_), rad_step(rad_step_)
{
    A = {(rad-rad_step)*sin(-ang_step), (rad-rad_step)*cos(-ang_step)-rad};
    B = {(rad-rad_step)*sin(ang_step), (rad-rad_step)*cos(ang_step)-rad};
    C = {(rad+rad_step)*sin(ang_step), (rad+rad_step)*cos(ang_step)-rad};
    D = {(rad+rad_step)*sin(-ang_step), (rad+rad_step)*cos(-ang_step)-rad};
    poly << A << B << C << D;
}
QRectF QGraphicsCellItem::boundingRect() const
{
    return QRectF(D,B);
}
void QGraphicsCellItem::paint(QPainter *painter, const QStyleOptionGraphicsItem *option, QWidget *widget)
{
    //painter->setPen(free_pen);
    painter->setBrush(current_brush);
    painter->setOpacity(opacity);
    painter->drawConvexPolygon(poly);
}
void QGraphicsCellItem::setOccupiedColor(int level)
{
    if (level >= 0 and level < 4)
    {
        current_brush = occupied_brushes[level];
        opacity = 1;
        update();
    }
}
void QGraphicsCellItem::setFreeColor()
{
    current_brush = free_brush;;
    opacity = 0.1;
    update();
}
void QGraphicsCellItem::mousePressEvent(QGraphicsSceneMouseEvent *event)
{
    update();
    QGraphicsItem::mousePressEvent(event);
}
void QGraphicsCellItem::mouseReleaseEvent(QGraphicsSceneMouseEvent *event)
{
    update();
    QGraphicsItem::mouseReleaseEvent(event);
}