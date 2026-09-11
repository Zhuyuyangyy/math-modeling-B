"""Conservative whole-cell absence certificates; no simulator access.

Every retained 25m square intersects the target disk. Q4 excludes a square
only if ALL its corners lie strictly inside the convex hull of SAME negative
stations, each within 1000m of the ENTIRE square. This handles unions across
triangle edges without mistaking a center-point sample for a certificate.
"""
import math
import numpy as np


def distance(a, b):
    return math.hypot(a[0]-b[0], a[1]-b[1])


def enclosing_circle(points):
    """Exact candidate enumeration in 2D, with conservative validation."""
    pts = list(dict.fromkeys(tuple(p) for p in points))
    best_c = pts[0]
    best_r = max(distance(best_c, p) for p in pts)
    def consider(c):
        nonlocal best_c, best_r
        r = max(distance(c, p) for p in pts)
        if r < best_r:
            best_c, best_r = c, r
    for i, a in enumerate(pts):
        for j in range(i):
            b = pts[j]
            consider(((a[0]+b[0])/2, (a[1]+b[1])/2))
            for k in range(j):
                c = pts[k]
                bx, by, cx, cy = b[0]-a[0], b[1]-a[1], c[0]-a[0], c[1]-a[1]
                det = 2*(bx*cy-by*cx)
                if abs(det) < 1e-10:
                    continue
                bb, cc = bx*bx+by*by, cx*cx+cy*cy
                consider((a[0]+(cy*bb-by*cc)/det, a[1]+(bx*cc-cx*bb)/det))
    return best_c, best_r


def nearest_clear_point(points, current, radius=19.8):
    """Projection onto intersection of equal-radius disks around all vertices.

    Boundary optimum is on a smooth arc (radial projection), or two circles'
    intersection. Enumerating both plus feasible current gives the optimum.
    The 0.2m reserve exceeds numerical tolerances.
    """
    pts = list(dict.fromkeys(tuple(p) for p in points))
    c, r = enclosing_circle(pts)
    if r > radius:
        return None
    feasible = lambda q: all(distance(q, p) <= radius+1e-8 for p in pts)
    if feasible(current):
        return tuple(current)
    candidates = [c]
    for i, a in enumerate(pts):
        d = distance(a, current)
        if d:
            q = (a[0]+radius*(current[0]-a[0])/d, a[1]+radius*(current[1]-a[1])/d)
            if feasible(q):
                candidates.append(q)
        for b in pts[:i]:
            d = distance(a, b)
            if not 1e-10 < d <= 2*radius:
                continue
            h = math.sqrt(max(0, radius*radius-d*d/4))
            mid = ((a[0]+b[0])/2, (a[1]+b[1])/2)
            for sign in (-1, 1):
                q = (mid[0]-sign*h*(b[1]-a[1])/d, mid[1]+sign*h*(b[0]-a[0])/d)
                if feasible(q):
                    candidates.append(q)
    return min(candidates, key=lambda q: distance(q, current))


class Certificates:
    def __init__(self, problem, step=25):
        self.problem = problem
        self.step, self.half = step, step/2
        axis = np.arange(-1800+step/2, 1800, step, dtype=float)
        xx, yy = np.meshgrid(axis, axis)
        centers = np.column_stack((xx.ravel(), yy.ravel()))
        nearest = np.maximum(np.abs(centers)-self.half, 0)
        self.centers = centers[(nearest**2).sum(axis=1) <= 1800**2+1e-8]
        self.offsets = np.array([[-1,-1],[1,-1],[1,1],[-1,1]])*self.half
        self.points = []
        self.point_ids = {}
        self.histories = {ch: frozenset() for ch in range(1,21)}
        self.cache = {frozenset(): np.zeros(len(self.centers), dtype=bool)}
        self.refined = {ch: np.zeros(len(self.centers), dtype=bool) for ch in range(1,21)}
        self.events = []

    def whole_disk(self, point, radius=1000):
        delta = np.abs(self.centers-np.asarray(point))+self.half
        return (delta**2).sum(axis=1) < (radius-1e-7)**2

    def excluded(self, ch):
        return self.cache[self.histories[ch]] | self.refined[ch]

    def remaining(self, ch):
        return ~self.excluded(ch)

    def absent(self, ch):
        return bool(self.excluded(ch).all())

    def negative(self, ch, point):
        # Use exact float coordinates: different measurements are not merged.
        point = tuple(float(x) for x in point)
        if point not in self.point_ids:
            self.point_ids[point] = len(self.points)
            self.points.append(point)
        pid = self.point_ids[point]
        old = self.histories[ch]
        key = old | {pid}
        if key not in self.cache:
            excluded = self.cache[old].copy()
            eligible = self.whole_disk(point) & ~excluded
            if self.problem == 3:
                excluded |= eligible
            elif len(key) >= 3 and eligible.any():
                ids = np.flatnonzero(eligible)
                centers = self.centers[ids]
                stations = np.asarray([self.points[k] for k in sorted(key)])
                delta = np.abs(centers[:,None,:]-stations[None,:,:])+self.half
                common = (delta**2).sum(axis=2) < (1000-1e-7)**2
                enough = common.sum(axis=1) >= 3
                local_ids = np.flatnonzero(enough)
                centers, common = centers[enough], common[enough]
                certified = np.ones(len(centers), dtype=bool)
                if len(centers):
                    count = common.sum(axis=1)
                    for offset in self.offsets:
                        vectors = stations[None,:,:]-(centers+offset)[:,None,:]
                        angles = np.arctan2(vectors[:,:,1], vectors[:,:,0])
                        angles = np.sort(np.where(common, angles, 10.0), axis=1)
                        gaps = np.diff(angles, axis=1)
                        gaps = np.where(np.arange(gaps.shape[1])[None,:] < count[:,None]-1, gaps, 0)
                        wrap = angles[:,0]+2*math.pi-angles[np.arange(len(centers)),count-1]
                        largest = np.maximum(gaps.max(axis=1), wrap)
                        certified &= largest < math.pi-1e-10
                    excluded[ids[local_ids[certified]]] = True
            self.cache[key] = excluded
        self.histories[ch] = key

    def positive(self, ch, point, angle=None):
        p = np.asarray(point)
        nearest = np.maximum(np.abs(self.centers-p)-self.half, 0)
        self.refined[ch] |= (nearest**2).sum(axis=1) > (1500+1e-7)**2
        if angle is not None:
            eps = math.radians(1.0051)
            lo, hi = math.radians(angle)-eps, math.radians(angle)+eps
            for normal in [(math.sin(lo),-math.cos(lo)),(-math.sin(hi),math.cos(hi))]:
                n = np.asarray(normal)
                smallest = (self.centers-p)@n-self.half*np.abs(n).sum()
                self.refined[ch] |= smallest > 1e-7

    def failed_clear(self, ch, point):
        self.refined[ch] |= self.whole_disk(point,20)

    def can_receive(self, ch, point):
        # Use 1500, NOT 1000: beyond minimum range signals can still exist.
        cells = self.centers[self.remaining(ch)]
        nearest = np.maximum(np.abs(cells-np.asarray(point))-self.half, 0)
        return bool(((nearest**2).sum(axis=1) <= (1500+1e-7)**2).any())

    def novelty(self, ch, point):
        mask = self.whole_disk(point) & self.remaining(ch)
        if not mask.any():
            return 0
        if self.problem == 3:
            return int(mask.sum())
        history = self.histories[ch]
        # Ranking only: full-cell coverage by a new direction is valuable even
        # before three negative stations close a convex-hull certificate.
        if history and min(distance(point, self.points[k]) for k in history) < 325:
            return 0
        return int(mask.sum())

    def potential(self, ch, point):
        """Strict certificate-contributing count: remaining cells fully inside
        B(point,1000). No ranking guard. A site with potential==0 for every
        unresolved channel can never help close any certificate, so visiting it
        is provably wasted movement and it may be dropped from the route."""
        mask = self.whole_disk(point) & self.remaining(ch)
        return int(mask.sum())

    def save(self):
        return dict(step_m=self.step, cells=len(self.centers), negative_stations=self.points,
                    channels={str(ch):dict(negative_station_ids=sorted(self.histories[ch]),
                                          remaining_cells=int(self.remaining(ch).sum())) for ch in range(1,21)},
                    rule='whole-cell disk exclusion for Q3; common-neighbor convex hull contains all four corners for Q4')
