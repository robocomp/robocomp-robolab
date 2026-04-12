#pragma once

#include <QWidget>
#include <QPainter>
#include <QPainterPath>
#include <QElapsedTimer>
#include <deque>
#include <unordered_map>
#include <string>
#include <mutex>
#include <cmath>

namespace rc {

/**
 * Lightweight scrolling time-series plot.
 *
 * Usage:
 *   plot->add_series("sdf_mse", Qt::red);
 *   // in compute loop:
 *   plot->add_point("sdf_mse", value);
 *
 * The X axis is wall-clock seconds since the first point.
 * The Y axis auto-scales to fit visible data.
 * Multiple named series share the same axes.
 */
class TimeSeriesPlot : public QWidget
{
    Q_OBJECT

public:
    explicit TimeSeriesPlot(QWidget* parent = nullptr);

    /** Register a new series with the given display colour.
     *  If avg_window > 0 a companion "<name>_avg" series is auto-created
     *  with a lighter colour and the running average is updated on every add_point(). */
    void add_series(const std::string& name, QColor colour,
                    float line_width = 1.5f, int avg_window = 0);

    /** Append a sample to the named series (call from any thread).
     *  If a running-average companion exists it is updated automatically. */
    void add_point(const std::string& name, float value);

    /** How many seconds of history to display (default 30). */
    void set_visible_window(float seconds);

protected:
    void paintEvent(QPaintEvent* event) override;
    void timerEvent(QTimerEvent* event) override;

private:
    struct Sample { float t; float v; };  // elapsed seconds, value

    struct Series
    {
        std::string name;
        QColor colour{Qt::white};
        float line_width = 1.5f;
        std::deque<Sample> samples;

        // Running-average state (0 = disabled)
        int avg_window = 0;
        std::string avg_companion;        // key of the companion avg series
        std::deque<float> avg_ring;       // last N raw values
        float avg_sum = 0.f;
    };

    std::mutex mu_;
    std::unordered_map<std::string, Series> series_;
    QElapsedTimer clock_;
    float window_sec_ = 30.f;

    // Margins (pixels)
    static constexpr int kLeft = 55;
    static constexpr int kRight = 10;
    static constexpr int kTop = 20;
    static constexpr int kBottom = 22;

    void draw_axes(QPainter& p, float t_min, float t_max, float v_min, float v_max) const;
    void draw_legend(QPainter& p) const;
};

} // namespace rc
