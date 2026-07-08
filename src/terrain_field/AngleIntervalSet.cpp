#include "AngleIntervalSet.hpp"

#include <algorithm>
#include <cmath>

namespace traversability_generator3d
{
namespace terrain_field
{

namespace
{
const double TWO_PI = 2.0 * M_PI;
const double EPS = 1e-12;

double normalize(double a)
{
    double v = std::fmod(a, TWO_PI);
    if (v < 0.0)
        v += TWO_PI;
    // guard against fmod returning TWO_PI due to rounding
    if (v >= TWO_PI)
        v -= TWO_PI;
    return v;
}
}

AngleIntervalSet AngleIntervalSet::full()
{
    AngleIntervalSet s;
    s.mArcs.emplace_back(0.0, TWO_PI);
    return s;
}

AngleIntervalSet AngleIntervalSet::empty()
{
    return AngleIntervalSet();
}

void AngleIntervalSet::add(double start, double width)
{
    if (width <= 0.0)
        return;
    if (width >= TWO_PI - EPS)
    {
        mArcs.clear();
        mArcs.emplace_back(0.0, TWO_PI);
        return;
    }
    const double s = normalize(start);
    const double e = s + width;
    if (e <= TWO_PI)
    {
        mArcs.emplace_back(s, e);
    }
    else
    {
        mArcs.emplace_back(s, TWO_PI);
        mArcs.emplace_back(0.0, e - TWO_PI);
    }
    normalizeAndMerge();
}

void AngleIntervalSet::normalizeAndMerge()
{
    if (mArcs.empty())
        return;
    std::sort(mArcs.begin(), mArcs.end());
    std::vector<std::pair<double, double>> merged;
    merged.reserve(mArcs.size());
    for (const auto& a : mArcs)
    {
        if (!merged.empty() && a.first <= merged.back().second + EPS)
        {
            merged.back().second = std::max(merged.back().second, a.second);
        }
        else
        {
            merged.push_back(a);
        }
    }
    mArcs.swap(merged);
}

AngleIntervalSet AngleIntervalSet::complement() const
{
    AngleIntervalSet out;
    double cursor = 0.0;
    for (const auto& a : mArcs)
    {
        if (a.first > cursor + EPS)
            out.mArcs.emplace_back(cursor, a.first);
        cursor = std::max(cursor, a.second);
    }
    if (cursor < TWO_PI - EPS)
        out.mArcs.emplace_back(cursor, TWO_PI);
    return out;
}

AngleIntervalSet AngleIntervalSet::unite(const AngleIntervalSet& other) const
{
    AngleIntervalSet out;
    out.mArcs = mArcs;
    out.mArcs.insert(out.mArcs.end(), other.mArcs.begin(), other.mArcs.end());
    out.normalizeAndMerge();
    return out;
}

AngleIntervalSet AngleIntervalSet::intersect(const AngleIntervalSet& other) const
{
    // sorted two-pointer sweep over non-wrapping arcs
    AngleIntervalSet out;
    size_t i = 0, j = 0;
    while (i < mArcs.size() && j < other.mArcs.size())
    {
        const double lo = std::max(mArcs[i].first, other.mArcs[j].first);
        const double hi = std::min(mArcs[i].second, other.mArcs[j].second);
        if (hi > lo + EPS)
            out.mArcs.emplace_back(lo, hi);
        if (mArcs[i].second < other.mArcs[j].second)
            ++i;
        else
            ++j;
    }
    return out;
}

bool AngleIntervalSet::contains(double angle) const
{
    const double a = normalize(angle);
    // arcs are sorted; binary search for the last arc starting at or before a
    size_t lo = 0, hi = mArcs.size();
    while (lo < hi)
    {
        const size_t mid = (lo + hi) / 2;
        if (mArcs[mid].first <= a + EPS)
            lo = mid + 1;
        else
            hi = mid;
    }
    if (lo > 0 && a <= mArcs[lo - 1].second + EPS)
        return true;
    // a == 0 may belong to an arc ending exactly at 2*pi (split wrap)
    if (a < EPS && !mArcs.empty() && mArcs.back().second >= TWO_PI - EPS)
        return true;
    return false;
}

bool AngleIntervalSet::isEmpty() const
{
    return totalWidth() < EPS;
}

bool AngleIntervalSet::isFull() const
{
    return totalWidth() > TWO_PI - 1e-9;
}

double AngleIntervalSet::totalWidth() const
{
    double sum = 0.0;
    for (const auto& a : mArcs)
        sum += a.second - a.first;
    return sum;
}

}
}
