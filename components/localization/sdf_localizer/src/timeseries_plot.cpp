#include "timeseries_plot.h"
#include <algorithm>
#include <cmath>
#include <limits>

namespace rc {

TimeSeriesPlot::TimeSeriesPlot(QWidget* parent)
    : QWidget(parent)
{
    setMinimumHeight(80);
    setAutoFillBackground(true);
    QPalette pal = palette();
    pal.setColor(QPalette::Window, QColor(30, 30, 30));
    setPalette(pal);

    clock_.start();
    startTimer(100);  // repaint at ~10 Hz
}

void TimeSeriesPlot::add_series(const std::string& name, QColor colour,
                                float line_width, int avg_window)
{
    std::lock_guard lk(mu_);
    auto& s = series_[name];
    s.name = name;
    s.colour = colour;
    s.line_width = line_width;
    s.avg_window = std::max(0, avg_window);

    if (s.avg_window > 0)
    {
        // Create a lighter, thinner companion series for the running average
        const std::string avg_name = name + "_avg";
        s.avg_companion = avg_name;
        auto& a = series_[avg_name];
        a.name = avg_name;
        a.colour = colour.lighter(160);
        a.line_width = line_width + 0.5f;
    }
}

void TimeSeriesPlot::add_point(const std::string& name, float value)
{
    std::lock_guard lk(mu_);
    auto it = series_.find(name);
    if (it == series_.end()) return;

    const float t = clock_.elapsed() / 1000.f;
    it->second.samples.push_back({t, value});

    // Trim old samples beyond 2× visible window
    const float cutoff = t - 2.f * window_sec_;
    auto& q = it->second.samples;
    while (!q.empty() && q.front().t < cutoff)
        q.pop_front();

    // Update running average companion if configured
    auto& s = it->second;
    if (s.avg_window > 0 && !s.avg_companion.empty())
    {
        s.avg_ring.push_back(value);
        s.avg_sum += value;
        while (static_cast<int>(s.avg_ring.size()) > s.avg_window)
        {
            s.avg_sum -= s.avg_ring.front();
            s.avg_ring.pop_front();
        }
        const float avg = s.avg_sum / static_cast<float>(s.avg_ring.size());

        auto avg_it = series_.find(s.avg_companion);
        if (avg_it != series_.end())
        {
            avg_it->second.samples.push_back({t, avg});
            auto& aq = avg_it->second.samples;
            while (!aq.empty() && aq.front().t < cutoff)
                aq.pop_front();
        }
    }
}

void TimeSeriesPlot::set_visible_window(float seconds)
{
    window_sec_ = std::max(1.f, seconds);
}

void TimeSeriesPlot::timerEvent(QTimerEvent*)
{
    update();  // schedule repaint
}

void TimeSeriesPlot::paintEvent(QPaintEvent*)
{
    QPainter p(this);
    p.setRenderHint(QPainter::Antialiasing);

    std::lock_guard lk(mu_);

    // Determine time range
    const float t_now = clock_.elapsed() / 1000.f;
    const float t_min = t_now - window_sec_;
    const float t_max = t_now;

    // Determine value range across all visible series
    float v_min = std::numeric_limits<float>::max();
    float v_max = std::numeric_limits<float>::lowest();
    for (const auto& [_, s] : series_)
    {
        for (const auto& pt : s.samples)
        {
            if (pt.t < t_min) continue;
            v_min = std::min(v_min, pt.v);
            v_max = std::max(v_max, pt.v);
        }
    }
    if (v_min >= v_max) { v_min = 0.f; v_max = 1.f; }

    // Add 5% padding
    const float pad = (v_max - v_min) * 0.05f;
    v_min -= pad;
    v_max += pad;

    draw_axes(p, t_min, t_max, v_min, v_max);

    // Plot area
    const float pw = static_cast<float>(width()  - kLeft - kRight);
    const float ph = static_cast<float>(height() - kTop  - kBottom);

    auto map_x = [&](float t) -> float { return kLeft + (t - t_min) / (t_max - t_min) * pw; };
    auto map_y = [&](float v) -> float { return kTop  + (1.f - (v - v_min) / (v_max - v_min)) * ph; };

    // Draw each series
    for (const auto& [_, s] : series_)
    {
        if (s.samples.empty()) continue;

        QPainterPath path;
        bool started = false;
        for (const auto& pt : s.samples)
        {
            if (pt.t < t_min) continue;
            const float sx = map_x(pt.t);
            const float sy = map_y(pt.v);
            if (!started) { path.moveTo(sx, sy); started = true; }
            else          { path.lineTo(sx, sy); }
        }
        p.setPen(QPen(s.colour, s.line_width));
        p.drawPath(path);
    }

    draw_legend(p);
}

void TimeSeriesPlot::draw_axes(QPainter& p, float t_min, float t_max,
                                float v_min, float v_max) const
{
    const float pw = static_cast<float>(width()  - kLeft - kRight);
    const float ph = static_cast<float>(height() - kTop  - kBottom);

    p.setPen(QPen(QColor(100, 100, 100), 1));
    // Y axis
    p.drawLine(kLeft, kTop, kLeft, kTop + static_cast<int>(ph));
    // X axis
    p.drawLine(kLeft, kTop + static_cast<int>(ph),
               kLeft + static_cast<int>(pw), kTop + static_cast<int>(ph));

    p.setFont(QFont("Monospace", 7));
    p.setPen(QColor(160, 160, 160));

    // Y tick labels (5 ticks)
    for (int i = 0; i <= 4; ++i)
    {
        const float frac = static_cast<float>(i) / 4.f;
        const float val = v_max - frac * (v_max - v_min);
        const int y = kTop + static_cast<int>(frac * ph);
        p.drawText(QRect(0, y - 7, kLeft - 4, 14), Qt::AlignRight | Qt::AlignVCenter,
                   QString::number(val, 'f', 3));
        // grid line
        p.setPen(QPen(QColor(60, 60, 60), 1, Qt::DotLine));
        p.drawLine(kLeft, y, kLeft + static_cast<int>(pw), y);
        p.setPen(QColor(160, 160, 160));
    }

    // X tick labels — show relative seconds
    for (int i = 0; i <= 4; ++i)
    {
        const float frac = static_cast<float>(i) / 4.f;
        const float t = t_min + frac * (t_max - t_min);
        const int x = kLeft + static_cast<int>(frac * pw);
        p.drawText(QRect(x - 20, kTop + static_cast<int>(ph) + 2, 40, 16),
                   Qt::AlignHCenter | Qt::AlignTop,
                   QString::number(t, 'f', 0) + "s");
    }
}

void TimeSeriesPlot::draw_legend(QPainter& p) const
{
    // Collect primary series (skip auto-generated "_avg" companions)
    struct Entry { std::string label; QColor colour; float last_val; };
    std::vector<Entry> entries;
    for (const auto& [key, s] : series_)
    {
        if (key.size() > 4 && key.substr(key.size() - 4) == "_avg") continue;
        float val = s.samples.empty() ? 0.f : s.samples.back().v;
        entries.push_back({s.name, s.colour, val});
    }
    if (entries.empty()) return;

    const QFont font("Monospace", 8);
    p.setFont(font);
    const QFontMetrics fm(font);

    constexpr int swatch = 12;
    constexpr int pad = 4;
    constexpr int row_h = 16;

    // Compute box width from longest label
    int max_text_w = 0;
    for (const auto& e : entries)
    {
        QString txt = QString::fromStdString(e.label) + QString(": %1").arg(e.last_val, 0, 'f', 4);
        max_text_w = std::max(max_text_w, fm.horizontalAdvance(txt));
    }
    const int box_w = swatch + pad * 3 + max_text_w;
    const int box_h = pad * 2 + static_cast<int>(entries.size()) * row_h;

    const int bx = kLeft + 6;
    const int by = kTop + 2;

    // Semi-transparent background
    p.setPen(Qt::NoPen);
    p.setBrush(QColor(30, 30, 30, 180));
    p.drawRoundedRect(bx, by, box_w, box_h, 3, 3);

    int y = by + pad;
    for (const auto& e : entries)
    {
        // Colour swatch
        p.setPen(Qt::NoPen);
        p.setBrush(e.colour);
        p.drawRect(bx + pad, y + 2, swatch, swatch);

        // Label + value
        p.setPen(QColor(220, 220, 220));
        QString txt = QString::fromStdString(e.label) + QString(": %1").arg(e.last_val, 0, 'f', 4);
        p.drawText(bx + pad + swatch + pad, y, max_text_w, row_h,
                   Qt::AlignLeft | Qt::AlignVCenter, txt);
        y += row_h;
    }
}

} // namespace rc
