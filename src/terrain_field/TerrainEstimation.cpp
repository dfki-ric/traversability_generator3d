#include "TerrainEstimation.hpp"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <deque>
#include <limits>

namespace traversability_generator3d
{
namespace terrain_field
{

namespace
{

const uint16_t UNASSIGNED_LAYER = std::numeric_limits<uint16_t>::max();
const std::size_t NO_INDEX = std::numeric_limits<std::size_t>::max();

/** One SUPPORT patch as tracked through layer building and fitting. */
struct SupportRef
{
    std::size_t cell;       //!< flat (row-major) cell index
    float mean;             //!< patch mean height [m]
    float variance;         //!< patch height variance [m^2]
    Eigen::Vector3f normal; //!< patch normal (z-up)
    uint16_t layer;         //!< assigned layer id (UNASSIGNED_LAYER while growing)
};

/** One sample entering a per-(cell, layer) fit. Coordinates are cell-center
 *  offsets relative to the query cell, so the fitted plane offset is directly
 *  the height at the query cell center. */
struct FitPoint
{
    double x, y;            //!< cell-center offset from the query cell [m]
    double z;               //!< patch mean height [m]
    double w;               //!< base weight: measurement precision x spatial
    Eigen::Vector3f normal; //!< patch normal, used by the low-count fallback
};

/** §5.2 triage: STRUCTURE patches carry vertical surface or excessive thickness
 *  and are excluded from all ground fitting and layer building. */
bool isStructure(const PatchSample& p, const TerrainParams& params)
{
    return std::abs(p.normal.z()) < params.structureNormalZ ||
           (double)(p.zMax - p.zMin) > 2.0 * params.maxStepHeight;
}

double median(std::vector<double> v)
{
    if (v.empty())
        return 0.0;
    std::sort(v.begin(), v.end());
    const std::size_t n = v.size();
    return (n % 2 == 1) ? v[n / 2] : 0.5 * (v[n / 2 - 1] + v[n / 2]);
}

/** Weighted LSQ plane z = a*x + b*y + c through @p pts with weights @p w,
 *  solved about the weighted centroid for conditioning. Returns false when the
 *  weighted xy spread is (near-)degenerate — i.e. fewer than three effective
 *  points or all of them (nearly) collinear — and no unique plane exists. */
bool fitPlaneWeighted(const std::vector<FitPoint>& pts, const std::vector<double>& w,
                      double& a, double& b, double& c)
{
    double sw = 0.0, mx = 0.0, my = 0.0, mz = 0.0;
    for (std::size_t i = 0; i < pts.size(); ++i)
    {
        sw += w[i];
        mx += w[i] * pts[i].x;
        my += w[i] * pts[i].y;
        mz += w[i] * pts[i].z;
    }
    if (sw <= 0.0)
        return false;
    mx /= sw;
    my /= sw;
    mz /= sw;

    double sxx = 0.0, sxy = 0.0, syy = 0.0, sxz = 0.0, syz = 0.0;
    for (std::size_t i = 0; i < pts.size(); ++i)
    {
        const double dx = pts[i].x - mx;
        const double dy = pts[i].y - my;
        const double dz = pts[i].z - mz;
        sxx += w[i] * dx * dx;
        sxy += w[i] * dx * dy;
        syy += w[i] * dy * dy;
        sxz += w[i] * dx * dz;
        syz += w[i] * dy * dz;
    }

    // rank test on the normalized xy scatter: the smallest eigenvalue vanishes
    // for collinear (or fewer than 3 distinct) sample positions
    const double cxx = sxx / sw, cxy = sxy / sw, cyy = syy / sw;
    const double t = 0.5 * (cxx + cyy);
    const double d = std::sqrt(std::max(0.0, 0.25 * (cxx - cyy) * (cxx - cyy) + cxy * cxy));
    if (t - d < 1e-8)
        return false;

    const double det = sxx * syy - sxy * sxy;
    a = (syy * sxz - sxy * syz) / det;
    b = (sxx * syz - sxy * sxz) / det;
    c = mz - a * mx - b * my;
    return true;
}

/** §5.1 robust fit for one (cell, layer): weighted LSQ plane, one Tukey/MAD
 *  robust reweight, refit; weighted-mean fallback below three points or on
 *  degenerate geometry. */
TerrainCell fitCellLayer(const std::vector<FitPoint>& pts, uint16_t layerId,
                         bool hasOwn, const TerrainParams& params)
{
    TerrainCell tc;
    tc.layerId = layerId;
    const std::size_t n = pts.size();

    std::vector<double> w(n);
    for (std::size_t i = 0; i < n; ++i)
        w[i] = pts[i].w;

    double a = 0.0, b = 0.0, c = 0.0;
    const bool planar = n >= 3 && fitPlaneWeighted(pts, w, a, b, c);

    if (planar)
    {
        // one robust reweight: Tukey biweight with c = 2.5 * 1.4826 * MAD
        std::vector<double> r(n);
        for (std::size_t i = 0; i < n; ++i)
            r[i] = pts[i].z - (a * pts[i].x + b * pts[i].y + c);
        const double med = median(r);
        std::vector<double> absDev(n);
        for (std::size_t i = 0; i < n; ++i)
            absDev[i] = std::abs(r[i] - med);
        const double mad = median(absDev);
        if (mad >= 1e-6)
        {
            const double cTukey = 2.5 * 1.4826 * mad;
            std::vector<double> wr(n);
            std::size_t alive = 0;
            for (std::size_t i = 0; i < n; ++i)
            {
                const double u = r[i] / cTukey;
                const double k = 1.0 - u * u;
                wr[i] = (std::abs(u) < 1.0) ? w[i] * k * k : 0.0;
                if (wr[i] > 0.0)
                    ++alive;
            }
            double a2, b2, c2;
            if (alive >= 3 && fitPlaneWeighted(pts, wr, a2, b2, c2))
            {
                a = a2;
                b = b2;
                c = c2;
                w.swap(wr);
            }
            // else: too few / degenerate survivors — keep the initial fit
        }

        double sw = 0.0, srr = 0.0;
        for (std::size_t i = 0; i < n; ++i)
        {
            const double rr = pts[i].z - (a * pts[i].x + b * pts[i].y + c);
            sw += w[i];
            srr += w[i] * rr * rr;
        }
        tc.height = (float)c; // plane at the query cell center (x = y = 0)
        Eigen::Vector3f nrm((float)-a, (float)-b, 1.f);
        nrm.normalize(); // z component positive by construction: z-up
        tc.normal = nrm;
        tc.roughness = (sw > 0.0) ? (float)std::sqrt(srr / sw) : 0.f;
    }
    else
    {
        // fewer than 3 points, or collinear sample positions: weighted mean
        // height with the weighted mean of the patch normals
        double sw = 0.0, sz = 0.0;
        Eigen::Vector3f sn = Eigen::Vector3f::Zero();
        for (std::size_t i = 0; i < n; ++i)
        {
            sw += w[i];
            sz += w[i] * pts[i].z;
            sn += (float)w[i] * pts[i].normal;
        }
        const double h = (sw > 0.0) ? sz / sw : 0.0;
        double srr = 0.0;
        for (std::size_t i = 0; i < n; ++i)
        {
            const double rr = pts[i].z - h;
            srr += w[i] * rr * rr;
        }
        tc.height = (float)h;
        tc.roughness = (sw > 0.0) ? (float)std::sqrt(srr / sw) : 0.f;
        if (sn.z() < 0.f)
            sn = -sn; // z-up
        const float len = sn.norm();
        tc.normal = (len > 1e-9f) ? Eigen::Vector3f(sn / len)
                                  : Eigen::Vector3f(0.f, 0.f, 1.f);
    }

    tc.heightSigma = tc.roughness / (float)std::sqrt((double)std::max<std::size_t>(1, n));
    tc.confidence = (float)(std::min(1.0, (double)n / 5.0) *
                            std::exp(-(double)tc.roughness / params.maxStepHeight));
    tc.flags = hasOwn ? TerrainCell::GROUND : TerrainCell::INTERPOLATED;
    if (n < 3)
        tc.flags |= TerrainCell::SPARSE;
    return tc;
}

const std::vector<PatchSample> NO_PATCHES;

}

void estimateTerrain(const TerrainGridInput& in, const TerrainParams& params,
                     TerrainGridOutput& out)
{
    const std::size_t W = in.width;
    const std::size_t H = in.height;
    out.width = W;
    out.height = H;
    out.cells.clear();
    out.cells.resize(W * H);
    if (W == 0 || H == 0)
        return;

    const std::size_t numCells = W * H;
    auto patchesAt = [&](std::size_t ci) -> const std::vector<PatchSample>& {
        return (ci < in.cells.size()) ? in.cells[ci] : NO_PATCHES;
    };

    // ---- 1) triage: keep SUPPORT patches only (STRUCTURE is consumed by L2) ----
    std::vector<SupportRef> supports;
    std::vector<std::vector<std::size_t>> cellSupports(numCells);
    for (std::size_t ci = 0; ci < numCells; ++ci)
    {
        for (const PatchSample& p : patchesAt(ci))
        {
            if (isStructure(p, params))
                continue;
            cellSupports[ci].push_back(supports.size());
            supports.push_back(SupportRef{ci, p.mean, p.variance, p.normal,
                                          UNASSIGNED_LAYER});
        }
        // deterministic seeding order: grow lower surfaces first
        std::sort(cellSupports[ci].begin(), cellSupports[ci].end(),
                  [&](std::size_t l, std::size_t r) {
                      return supports[l].mean < supports[r].mean;
                  });
    }

    // ---- 2) layers: region-grow SUPPORT patches over 4-connected cells ----
    // Patches in adjacent cells connect iff |mean_a - mean_b| <= maxStepHeight;
    // a layer holds at most one patch per cell (same-cell patches never share a
    // layer), enforced with a per-layer cell stamp. Layer ids never repeat, so
    // the stamp array needs no clearing between layers.
    std::vector<uint32_t> stamp(numCells, std::numeric_limits<uint32_t>::max());
    uint32_t nextLayer = 0;
    const int dx4[4] = {1, -1, 0, 0};
    const int dy4[4] = {0, 0, 1, -1};
    for (std::size_t ci = 0; ci < numCells && nextLayer < UNASSIGNED_LAYER; ++ci)
    {
        for (std::size_t seed : cellSupports[ci])
        {
            if (supports[seed].layer != UNASSIGNED_LAYER)
                continue;
            if (nextLayer >= UNASSIGNED_LAYER)
                break; // id space exhausted; remaining patches stay unassigned
            const uint32_t L = nextLayer++;
            supports[seed].layer = (uint16_t)L;
            stamp[ci] = L;
            std::deque<std::size_t> queue(1, seed);
            while (!queue.empty())
            {
                const std::size_t cur = queue.front();
                queue.pop_front();
                const std::size_t cc = supports[cur].cell;
                const int cx = (int)(cc % W);
                const int cy = (int)(cc / W);
                const float m = supports[cur].mean;
                for (int k = 0; k < 4; ++k)
                {
                    const int nx = cx + dx4[k];
                    const int ny = cy + dy4[k];
                    if (nx < 0 || ny < 0 || nx >= (int)W || ny >= (int)H)
                        continue;
                    const std::size_t nc = (std::size_t)ny * W + (std::size_t)nx;
                    if (stamp[nc] == L)
                        continue; // this layer already holds a patch here
                    // the height-closest unassigned support within the step
                    // limit joins the layer
                    std::size_t best = NO_INDEX;
                    float bestDelta = 0.f;
                    for (std::size_t cand : cellSupports[nc])
                    {
                        if (supports[cand].layer != UNASSIGNED_LAYER)
                            continue;
                        const float delta = std::abs(supports[cand].mean - m);
                        if (delta <= (float)params.maxStepHeight &&
                            (best == NO_INDEX || delta < bestDelta))
                        {
                            best = cand;
                            bestDelta = delta;
                        }
                    }
                    if (best != NO_INDEX)
                    {
                        supports[best].layer = (uint16_t)L;
                        stamp[nc] = L;
                        queue.push_back(best);
                    }
                }
            }
        }
    }

    // ---- 3) robust per-(cell, layer) fit over the 3x3 neighborhood ----
    auto supportOfLayer = [&](std::size_t ci, uint16_t L) -> const SupportRef* {
        for (std::size_t s : cellSupports[ci])
            if (supports[s].layer == L)
                return &supports[s];
        return nullptr;
    };

    const double res = params.gridResolution;
    const double sigma0Sq = params.sigma0 * params.sigma0;

    for (std::size_t y = 0; y < H; ++y)
    {
        for (std::size_t x = 0; x < W; ++x)
        {
            const std::size_t ci = y * W + x;

            // candidate layers: the cell's own layers; for hole cells (no own
            // SUPPORT patch) instead the neighborhood layers carried by at
            // least two neighbor cells (-> INTERPOLATED, bounded to one cell)
            std::vector<uint16_t> candidates;
            for (std::size_t s : cellSupports[ci])
                if (supports[s].layer != UNASSIGNED_LAYER)
                    candidates.push_back(supports[s].layer);
            const bool hasOwnSupport = !candidates.empty();
            if (!hasOwnSupport)
            {
                std::vector<std::pair<uint16_t, int>> carriers;
                for (int jy = (int)y - 1; jy <= (int)y + 1; ++jy)
                {
                    for (int jx = (int)x - 1; jx <= (int)x + 1; ++jx)
                    {
                        if (jx < 0 || jy < 0 || jx >= (int)W || jy >= (int)H)
                            continue;
                        const std::size_t cj = (std::size_t)jy * W + (std::size_t)jx;
                        if (cj == ci)
                            continue;
                        // a layer occurs at most once per cell, so this counts
                        // carrier *cells*
                        for (std::size_t s : cellSupports[cj])
                        {
                            const uint16_t L = supports[s].layer;
                            if (L == UNASSIGNED_LAYER)
                                continue;
                            bool found = false;
                            for (auto& c : carriers)
                            {
                                if (c.first == L)
                                {
                                    ++c.second;
                                    found = true;
                                    break;
                                }
                            }
                            if (!found)
                                carriers.emplace_back(L, 1);
                        }
                    }
                }
                for (const auto& c : carriers)
                    if (c.second >= 2)
                        candidates.push_back(c.first);
            }
            std::sort(candidates.begin(), candidates.end());
            candidates.erase(std::unique(candidates.begin(), candidates.end()),
                             candidates.end());

            for (uint16_t L : candidates)
            {
                std::vector<FitPoint> pts;
                pts.reserve(9);
                bool own = false;
                for (int jy = (int)y - 1; jy <= (int)y + 1; ++jy)
                {
                    for (int jx = (int)x - 1; jx <= (int)x + 1; ++jx)
                    {
                        if (jx < 0 || jy < 0 || jx >= (int)W || jy >= (int)H)
                            continue;
                        const std::size_t cj = (std::size_t)jy * W + (std::size_t)jx;
                        const SupportRef* s = supportOfLayer(cj, L);
                        if (!s)
                            continue;
                        FitPoint p;
                        // offsets between cell centers (ix + 0.5) * res
                        p.x = ((double)jx - (double)x) * res;
                        p.y = ((double)jy - (double)y) * res;
                        p.z = (double)s->mean;
                        const double spatial = (cj == ci) ? 1.0 : params.neighborWeight;
                        p.w = spatial / ((double)s->variance + sigma0Sq);
                        p.normal = s->normal;
                        if (cj == ci)
                            own = true;
                        pts.push_back(p);
                    }
                }
                if (pts.empty())
                    continue;
                out.cells[ci].push_back(fitCellLayer(pts, L, own, params));
            }
        }
    }
}

}
}
