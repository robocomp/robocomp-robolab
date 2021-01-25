//
// Created by salabeta on 24/11/20.
//

#ifndef GOTOXY_GRID_H
#define GOTOXY_GRID_H
#include <functional>
#include <QGraphicsItem>
#include <collisions.h>

template<typename HMIN, HMIN hmin_, typename WIDTH, WIDTH width_, typename TILE, TILE tile_>
class Grid
{
    int hmin, width, tile;
    public:
        Grid()
        {
            hmin = hmin_; width = width_; tile = tile_;
            array.resize((int)(width/tile));
            for (auto &row : array)
                row.resize((int)(width/tile));
            int k=0;
            for (int i = hmin; i < width/2; i += tile, k++)
            {
                int l=0;
                for (int j = hmin; j < width/2; j += tile, l++)
                {
                    array[k][l] = Value{false, nullptr, nullptr, i, j, k, l, -1};
                }
            }
        };

        struct Value
        {
            bool occupied = false;
            QGraphicsRectItem * paint_cell = nullptr;
            QGraphicsTextItem * text_cell = nullptr;
            int cx, cy;         //world coor
            int k,l;           //arrat coor
            int dist;       //dist vecinos
        };

        std::vector<std::vector<Value>> array;
        const std::vector<std::tuple<int, int>> neigh_coor{ {-1,-1}, {0,-1}, {1,-1}, {-1,0}, {1,0}, {-1,1}, {0,1}, {-1,1} };
        void initialize(std::shared_ptr<Collisions> collisions, QGraphicsScene *scene)
        {
            auto fondo = QColor("LightGreen"); fondo.setAlpha(40);
            QFont font("Bavaria");
            font.setPointSize(40);
            font.setWeight(QFont::TypeWriter);
            for (auto &row : array)
            {
                for (auto &elem : row)
                {
                    elem.paint_cell = scene->addRect(-tile / 2, -tile / 2, tile, tile, QPen(QColor("DarkGreen")), QBrush(fondo));
                    elem.paint_cell->setPos(elem.cx, elem.cy);
                    elem.text_cell = scene->addText("-1", font);
                    elem.text_cell->setPos(elem.cx - tile / 2, elem.cy - tile / 2);
                    // Get the current transform
                    QTransform transform(elem.text_cell->transform());
                    qreal m11 = transform.m11();    // Horizontal scaling
                    qreal m12 = transform.m12();    // Vertical shearing
                    qreal m13 = transform.m13();    // Horizontal Projection
                    qreal m21 = transform.m21();    // Horizontal shearing
                    qreal m22 = transform.m22();    // vertical scaling
                    qreal m23 = transform.m23();    // Vertical Projection
                    qreal m31 = transform.m31();    // Horizontal Position (DX)
                    qreal m32 = transform.m32();    // Vertical Position (DY)
                    qreal m33 = transform.m33();    // Addtional Projection Factor
                    // Vertical flip
                    m22 = -m22;
                    // Write back to the matrix
                    transform.setMatrix(m11, m12, m13, m21, m22, m23, m31, m32, m33);
                    // Set the items transformation
                    elem.text_cell->setTransform(transform);

                    // check obstacles
                    if (not collisions->checkRobotValidStateAtTargetFast(QVec::vec3(elem.cx, 10, elem.cy), QVec::zeros(3)))
                    {
                        elem.occupied = true;
                        QColor color("Red");
                        color.setAlpha(40);
                        elem.paint_cell->setBrush(QColor(color));
                    }
                }
            }
        }
        void set_occupied(int x, int z)
        {
            if(auto cell = transform(x,z); cell.has_value())
            {
                auto [i,j] = cell.value();
                array[i][j].paint_cell->setBrush(QColor("Red"));
                array[i][j].occupied = true;
            }
        }
        std::optional<Value> get_value(int x, int z)
        {
            if( auto cell = transform(x,z); cell.has_value())
            {
                auto [i,j] = cell.value();
                return this->array[i][j];
            }
            else
                return {};
        }
        std::optional<Value> get_value(const QPointF &p)
        {
            if( auto cell = transform(p.x(), p.y()); cell.has_value())
            {
                auto [i,j] = cell.value();
                return this->array[i][j];
            }
            else
                return {};
        }

        std::optional<std::tuple<int, int>> transform(int x, int y)
        {
            int k = (1.f/tile)*x + (width/tile)/2;
            int l = (1.f/tile)*y + (width/tile)/2;
            if(k>0 and k<(int)array.size() and l>0 and l < (int)array.size())
                return std::make_tuple(k,l);
            else
                return {};
        }
        std::vector<Value> neighboors(const Value &v, int dist)
        {
            std::vector<Value> selected;
            for(auto [dk, dl] : neigh_coor)
            {
                int k = v.k + dk;        // OJO hay que añadir al struct Value las coordenadas de array
                int l = v.l + dl;
                if( k<(int)array.size() and k>0 and l<(int)array.size() and l>0  and  not array[k][l].occupied and array[k][l].dist == -1)
                {
                    if( has_adjacent_obstacle(k,l))
                        array[k][l].dist = 100;
                    else
                        array[k][l].dist = dist;
                    array[k][l].text_cell->setPlainText(QString::number( array[k][l].dist));
                    selected.push_back(array[k][l]);
                }
            }
            return selected;
        }
        bool has_adjacent_obstacle(int k, int l)
        {
            for(auto [dk, dl] : neigh_coor)
            {
                int kk = k + dk;        // OJO hay que añadir al struct Value las coordenadas de array
                int ll = l + dl;
                if (kk < (int) array.size() and kk > 0 and ll < (int) array.size() and ll > 0 and array[kk][ll].occupied)
                    return true;
            }
            return false;
        }
        void reset_cell_distances()
        {
            for (auto &row : array)
                for (auto &elem : row)
                    elem.dist = -1;
        }
};


#endif //GOTOXY_GRID_H
