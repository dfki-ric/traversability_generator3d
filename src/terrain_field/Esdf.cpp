#include "Esdf.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

namespace traversability_generator3d
{
namespace terrain_field
{

namespace
{
// Large finite sentinel for "no source". Chosen so that adding any squared grid
// offset is absorbed (exact in double for offsets far beyond any realistic grid)
// and results carrying it stay orders of magnitude above every genuine distance.
const double INF_SENTINEL = 1e20;

/** 1D squared-distance transform (Felzenszwalb & Huttenlocher lower envelope of
 *  parabolas): d[q] = min_p ((q - p)^2 + f[p]) for q, p in [0, n).
 *  v (size >= n) and z (size >= n + 1) are scratch buffers. */
void dt1d(const std::vector<double>& f, std::vector<double>& d,
          std::vector<int>& v, std::vector<double>& z, std::size_t n)
{
    int k = 0;
    v[0] = 0;
    z[0] = -INF_SENTINEL;
    z[1] = INF_SENTINEL;
    for (std::size_t q = 1; q < n; ++q)
    {
        const double fq = f[q] + (double)(q * q);
        // intersection of parabola q with the rightmost envelope parabola v[k]
        double s = (fq - (f[v[k]] + (double)v[k] * (double)v[k])) /
                   (2.0 * (double)q - 2.0 * (double)v[k]);
        while (s <= z[k])
        {
            --k;
            s = (fq - (f[v[k]] + (double)v[k] * (double)v[k])) /
                (2.0 * (double)q - 2.0 * (double)v[k]);
        }
        ++k;
        v[k] = (int)q;
        z[k] = s;
        z[k + 1] = INF_SENTINEL;
    }
    k = 0;
    for (std::size_t q = 0; q < n; ++q)
    {
        while (z[k + 1] < (double)q)
            ++k;
        const double dq = (double)q - (double)v[k];
        d[q] = dq * dq + f[v[k]];
    }
}
}

void computeEsdfSquared(std::size_t w, std::size_t h,
                        const std::vector<std::uint8_t>& blocked,
                        std::vector<float>& outSquared)
{
    outSquared.assign(w * h, std::numeric_limits<float>::infinity());
    if (w == 0 || h == 0)
        return;

    const std::size_t n = std::max(w, h);
    std::vector<double> f(n);
    std::vector<double> d(n);
    std::vector<double> z(n + 1);
    std::vector<int> v(n);
    std::vector<double> colDist(w * h);

    // pass 1: per column, squared distance along y to the nearest blocked cell
    for (std::size_t x = 0; x < w; ++x)
    {
        for (std::size_t y = 0; y < h; ++y)
            f[y] = blocked[y * w + x] ? 0.0 : INF_SENTINEL;
        dt1d(f, d, v, z, h);
        for (std::size_t y = 0; y < h; ++y)
            colDist[y * w + x] = d[y];
    }

    // pass 2: per row, combine with squared distance along x
    // any result above the largest possible in-grid squared distance still
    // carries the sentinel, i.e. there is no blocked cell anywhere
    const double maxFinite =
        (double)((w - 1) * (w - 1)) + (double)((h - 1) * (h - 1));
    for (std::size_t y = 0; y < h; ++y)
    {
        for (std::size_t x = 0; x < w; ++x)
            f[x] = colDist[y * w + x];
        dt1d(f, d, v, z, w);
        for (std::size_t x = 0; x < w; ++x)
            outSquared[y * w + x] =
                d[x] > maxFinite ? std::numeric_limits<float>::infinity()
                                 : (float)d[x];
    }
}

void computeEsdf(std::size_t w, std::size_t h,
                 const std::vector<std::uint8_t>& blocked,
                 double resolution, std::vector<float>& outMeters)
{
    computeEsdfSquared(w, h, blocked, outMeters);
    for (float& value : outMeters)
    {
        if (!std::isinf(value))
            value = (float)(std::sqrt((double)value) * resolution);
    }
}

}
}
