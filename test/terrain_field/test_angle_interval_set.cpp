// Standalone brute-force verification of AngleIntervalSet.
// Build: g++ -O2 -std=c++14 -I ../../src test_angle_interval_set.cpp \
//            ../../src/terrain_field/AngleIntervalSet.cpp -o test_ais && ./test_ais

#include "terrain_field/AngleIntervalSet.hpp"

#include <cmath>
#include <cstdio>
#include <random>
#include <vector>

using traversability_generator3d::terrain_field::AngleIntervalSet;

namespace
{
const double TWO_PI = 2.0 * M_PI;

// Reference model: dense boolean bitmask over [0, 2*pi).
struct RefSet
{
    static const int N = 1 << 18; // ~24 microrad resolution
    std::vector<bool> bits;
    RefSet() : bits(N, false) {}

    static int idx(double a)
    {
        double v = std::fmod(a, TWO_PI);
        if (v < 0.0)
            v += TWO_PI;
        int i = (int)(v / TWO_PI * N);
        return std::min(std::max(i, 0), N - 1);
    }

    void add(double start, double width)
    {
        if (width <= 0.0)
            return;
        if (width >= TWO_PI)
        {
            std::fill(bits.begin(), bits.end(), true);
            return;
        }
        const int n = (int)std::ceil(width / TWO_PI * N);
        const int s = idx(start);
        for (int k = 0; k <= n; ++k)
            bits[(s + k) % N] = true;
    }
};

double cellCenter(int i)
{
    return (i + 0.5) / RefSet::N * TWO_PI;
}
}

int main()
{
    std::mt19937 rng(4242);
    std::uniform_real_distribution<double> ang(-3.0 * TWO_PI, 3.0 * TWO_PI);
    std::uniform_real_distribution<double> wid(0.0, 1.5 * TWO_PI);
    std::uniform_int_distribution<int> narcs(0, 8);

    const int CASES = 400;
    const double resolution = TWO_PI / RefSet::N;
    // membership checked at cell centers at least this far from any set boundary
    const double guard = 4.0 * resolution;

    long long checked = 0, mismatches = 0;
    int widthFails = 0, algebraFails = 0;

    for (int c = 0; c < CASES; ++c)
    {
        AngleIntervalSet a, b;
        RefSet ra, rb;
        const int na = narcs(rng), nb = narcs(rng);
        std::vector<std::pair<double, double>> arcsA, arcsB;
        for (int k = 0; k < na; ++k)
        {
            double s = ang(rng), w = wid(rng);
            a.add(s, w);
            ra.add(s, w);
            arcsA.emplace_back(s, w);
        }
        for (int k = 0; k < nb; ++k)
        {
            double s = ang(rng), w = wid(rng);
            b.add(s, w);
            rb.add(s, w);
            arcsB.emplace_back(s, w);
        }

        const AngleIntervalSet comp = a.complement();
        const AngleIntervalSet inter = a.intersect(b);
        const AngleIntervalSet uni = a.unite(b);

        // helper: distance of angle x to nearest canonical boundary of a set
        auto nearBoundary = [&](const AngleIntervalSet& s, double x)
        {
            for (const auto& arc : s.arcs())
            {
                for (double bnd : {arc.first, arc.second})
                {
                    double d = std::fabs(x - bnd);
                    d = std::min(d, TWO_PI - d);
                    if (d < guard)
                        return true;
                }
            }
            return false;
        };

        std::uniform_int_distribution<int> cell(0, RefSet::N - 1);
        for (int q = 0; q < 3000; ++q)
        {
            const int i = cell(rng);
            const double x = cellCenter(i);
            if (nearBoundary(a, x) || nearBoundary(b, x))
                continue;
            ++checked;
            const bool ea = ra.bits[i];
            const bool eb = rb.bits[i];
            if (a.contains(x) != ea) { ++mismatches; }
            if (comp.contains(x) != !ea) { ++mismatches; }
            if (inter.contains(x) != (ea && eb)) { ++mismatches; }
            if (uni.contains(x) != (ea || eb)) { ++mismatches; }
        }

        // width algebra: |A| + |~A| == 2*pi ; |A∩B| + |A∪B| == |A| + |B|
        if (std::fabs(a.totalWidth() + comp.totalWidth() - TWO_PI) > 1e-6)
            ++widthFails;
        if (std::fabs(inter.totalWidth() + uni.totalWidth() -
                      (a.totalWidth() + b.totalWidth())) > 1e-6)
            ++algebraFails;

        // full/empty invariants
        if (na == 0 && !a.isEmpty()) ++algebraFails;
        if (na == 0 && !comp.isFull()) ++algebraFails;
    }

    std::printf("cases:          %d\n", CASES);
    std::printf("point checks:   %lld\n", checked);
    std::printf("mismatches:     %lld\n", mismatches);
    std::printf("width fails:    %d\n", widthFails);
    std::printf("algebra fails:  %d\n", algebraFails);
    const bool pass = mismatches == 0 && widthFails == 0 && algebraFails == 0 && checked > 100000;
    std::printf("RESULT: %s\n", pass ? "PASS" : "FAIL");
    return pass ? 0 : 1;
}
