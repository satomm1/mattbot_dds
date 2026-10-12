# Vendored verbatim from https://github.com/satomm1/dds_robot_platform/blob/f4b34028e5b6c5ffe1c5789768989bc70df18f8d/dds/dds_utils/transform.py - do not edit; re-copy instead.
"""
Map-to-map transforms built from known point correspondences.

Two models share one interface (MapTransform.apply(point, forward)):
- RigidMapTransform: SE(2) Kabsch fit (estimate_se2).
- TpsMapTransform: thin plate spline warp; heading follows the local Jacobian
  and the inverse is solved exactly with Newton steps on the forward spline.

Only numpy is required so this file can be vendored on its own.
"""

import numpy as np

TPS_MIN_POINTS = 4
_CHUNK = 200_000
_NEWTON_ITERS = 8
_NEWTON_TOL = 1e-9


def load_known_points(path):
    """Read 'x,y' lines from a known-points file, skipping blank lines."""
    points = []
    with open(path, 'r') as f:
        for line in f:
            line = line.strip()
            if not line:
                continue
            x, y = line.split(',')
            points.append((float(x), float(y)))
    return points


def estimate_se2(points, ref_points):
    """
    Estimate the rigid 2D transform (R, t) that maps points onto ref_points
    (Kabsch / SVD), i.e. ref ~= R @ p + t.
    """
    if len(points) != len(ref_points):
        raise ValueError(f"known point count mismatch: {len(points)} vs {len(ref_points)}")
    if len(points) < 2:
        raise ValueError("need at least 2 known point pairs")

    if [tuple(p) for p in points] == [tuple(p) for p in ref_points]:
        return np.identity(2), np.zeros((2, 1))

    known_points = np.array(points, dtype=float)
    reference_known_points = np.array(ref_points, dtype=float)

    centroid1 = np.mean(known_points, axis=0)
    centroid2 = np.mean(reference_known_points, axis=0)
    centered_points1 = known_points - centroid1
    centered_points2 = reference_known_points - centroid2

    H = np.dot(centered_points1.T, centered_points2)
    U, S, Vt = np.linalg.svd(H)
    R = Vt.T @ U.T

    if np.linalg.det(R) < 0:
        Vt[1, :] *= -1
        R = Vt.T @ U.T

    t = centroid2 - R @ centroid1
    return R, t


def transform_se2(R, t, point, forward=True):
    if R is None:
        return point

    point_xy = np.array([point[0], point[1]])
    if forward:
        new_point_xy = R @ point_xy + t
        new_point_theta = point[2] + np.arctan2(R[1, 0], R[0, 0])
        return np.concatenate((new_point_xy, [new_point_theta]))
    new_point_xy = R.T @ (point_xy - t)
    new_point_theta = point[2] - np.arctan2(R[1, 0], R[0, 0])
    return np.concatenate((new_point_xy, [new_point_theta]))


# ---------------------------------------------------------------------------
# Thin plate spline
# ---------------------------------------------------------------------------

def _tps_kernel(r2):
    """U(r) = r^2 log r, written in terms of r^2 (0 at r = 0)."""
    with np.errstate(divide='ignore', invalid='ignore'):
        return np.where(r2 > 0, 0.5 * r2 * np.log(r2), 0.0)


class TPS2D:
    """2D thin plate spline mapping src points onto dst points."""

    def __init__(self, src, dst, smooth=0.0):
        src = np.asarray(src, dtype=float)
        dst = np.asarray(dst, dtype=float)
        n = len(src)
        if n < 3:
            raise ValueError('TPS needs at least 3 point pairs')
        if len(dst) != n:
            raise ValueError(f'known point count mismatch: {n} vs {len(dst)}')
        K = _tps_kernel(np.sum((src[:, None] - src[None]) ** 2, axis=-1))
        P = np.hstack([np.ones((n, 1)), src])
        A = np.zeros((n + 3, n + 3))
        A[:n, :n] = K + smooth * np.eye(n)
        A[:n, n:] = P
        A[n:, :n] = P.T
        b = np.zeros((n + 3, 2))
        b[:n] = dst
        try:
            sol = np.linalg.solve(A, b)
        except np.linalg.LinAlgError as exc:
            raise ValueError('TPS system is singular - are the points collinear or duplicated?') from exc
        if not np.all(np.isfinite(sol)):
            raise ValueError('TPS solve produced non-finite weights')
        self.src = src
        self.w = sol[:n]
        self.a = sol[n:]

    def __call__(self, xy):
        xy = np.atleast_2d(np.asarray(xy, dtype=float))
        out = np.empty_like(xy)
        for i in range(0, len(xy), _CHUNK):
            c = xy[i:i + _CHUNK]
            U = _tps_kernel(np.sum((c[:, None] - self.src[None]) ** 2, axis=-1))
            out[i:i + _CHUNK] = self.a[0] + c @ self.a[1:] + U @ self.w
        return out

    def jacobian(self, xy):
        """d f / d p at each point, shape (N, 2, 2) with J[k, j] = d f_k / d p_j."""
        xy = np.atleast_2d(np.asarray(xy, dtype=float))
        out = np.empty((len(xy), 2, 2))
        for i in range(0, len(xy), _CHUNK):
            c = xy[i:i + _CHUNK]
            d = c[:, None] - self.src[None]                       # (N, n, 2)
            r2 = np.sum(d ** 2, axis=-1)
            with np.errstate(divide='ignore', invalid='ignore'):
                g = np.where(r2 > 0, np.log(r2) + 1.0, 0.0)       # dU/dp = (log r^2 + 1)(p - c)
            grad = g[..., None] * d                               # (N, n, 2)
            out[i:i + _CHUNK] = self.a[1:].T[None] + np.einsum('nk,mnj->mkj', self.w, grad)
        return out

    def bending_energy(self):
        K = _tps_kernel(np.sum((self.src[:, None] - self.src[None]) ** 2, axis=-1))
        return float(np.trace(self.w.T @ K @ self.w))


# ---------------------------------------------------------------------------
# MapTransform models
# ---------------------------------------------------------------------------

def _split_pose(point):
    """Return (xy, theta-or-None) for a 2- or 3-element point."""
    p = np.asarray(point, dtype=float).ravel()
    return p[:2], (p[2] if len(p) > 2 else None)


def _join_pose(xy, theta):
    if theta is None:
        return np.asarray(xy, dtype=float)
    return np.concatenate((xy, [theta]))


def _heading_through(J, theta):
    """Push heading theta through the 2x2 linear map J."""
    v = J @ np.array([np.cos(theta), np.sin(theta)])
    return float(np.arctan2(v[1], v[0]))


def _flat_points(points):
    return [float(v) for p in points for v in p[:2]]


def _unflat_points(values):
    values = [float(v) for v in values]
    return [(values[i], values[i + 1]) for i in range(0, len(values) - 1, 2)]


class RigidMapTransform:
    """SE(2): ref = R @ p + t."""

    model = 'rigid'

    def __init__(self, R, t):
        self.R = np.asarray(R, dtype=float).reshape(2, 2)
        self.t = np.asarray(t, dtype=float).reshape(2)
        self.fallback_reason = None

    def apply(self, point, forward=True):
        xy, theta = _split_pose(point)
        out = transform_se2(self.R, self.t, (xy[0], xy[1], 0.0 if theta is None else theta), forward)
        return out if theta is not None else out[:2]

    def apply_xy(self, xy, forward=True):
        xy = np.atleast_2d(np.asarray(xy, dtype=float))
        if forward:
            return xy @ self.R.T + self.t
        return (xy - self.t) @ self.R

    def to_doc(self):
        return {
            'model': self.model,
            'R': self.R.flatten().tolist(),
            't': self.t.tolist(),
        }


class TpsMapTransform:
    """Thin plate spline from local points (src) onto reference points (dst)."""

    model = 'tps'

    def __init__(self, src, dst, smooth=0.0):
        self.src = [tuple(map(float, p[:2])) for p in src]
        self.dst = [tuple(map(float, p[:2])) for p in dst]
        self.smooth = float(smooth)
        self.fwd = TPS2D(self.src, self.dst, self.smooth)
        self._inv0 = TPS2D(self.dst, self.src, self.smooth)   # starting guess for the exact inverse
        self.R, self.t = estimate_se2(self.src, self.dst)     # rigid approximation, for legacy readers
        self.t = np.asarray(self.t, dtype=float).reshape(2)
        self.fallback_reason = None

    def apply_xy(self, xy, forward=True):
        """Vectorized position-only transform of (N, 2) points."""
        xy = np.atleast_2d(np.asarray(xy, dtype=float))
        if forward:
            return self.fwd(xy)
        return self._invert(xy)

    def _invert(self, q):
        p = self._inv0(q)
        for _ in range(_NEWTON_ITERS):
            r = self.fwd(p) - q
            if np.max(np.abs(r)) < _NEWTON_TOL:
                break
            J = self.fwd.jacobian(p)
            p = p - np.linalg.solve(J, r[..., None])[..., 0]
        return p

    def apply(self, point, forward=True):
        xy, theta = _split_pose(point)
        if forward:
            new_xy = self.fwd(xy)[0]
            J = self.fwd.jacobian(xy)[0]
        else:
            new_xy = self._invert(xy[None])[0]
            J = np.linalg.inv(self.fwd.jacobian(new_xy)[0])
        new_theta = None if theta is None else _heading_through(J, theta)
        return _join_pose(new_xy, new_theta)

    def to_doc(self):
        return {
            'model': self.model,
            'R': self.R.flatten().tolist(),
            't': self.t.tolist(),
            'src_points': _flat_points(self.src),
            'dst_points': _flat_points(self.dst),
            'smooth': self.smooth,
        }


def build_map_transform(points, ref_points, mode='tps', smooth=0.0, min_points=TPS_MIN_POINTS):
    """
    Build the transform from local known points to reference known points.

    mode='tps' uses a thin plate spline when there are >= min_points pairs and the
    fit succeeds; otherwise (or mode='rigid') returns the SE(2) fit. When a TPS was
    requested but not used, the result's fallback_reason says why.
    """
    R, t = estimate_se2(points, ref_points)
    if [tuple(p) for p in points] == [tuple(p) for p in ref_points]:
        return RigidMapTransform(R, t)
    if mode != 'tps':
        return RigidMapTransform(R, t)

    reason = None
    if len(points) < min_points:
        reason = f'only {len(points)} point pairs (TPS needs >= {min_points})'
    else:
        try:
            return TpsMapTransform(points, ref_points, smooth)
        except (ValueError, np.linalg.LinAlgError) as exc:
            reason = f'TPS fit failed: {exc}'
    tf = RigidMapTransform(R, t)
    tf.fallback_reason = reason
    return tf


def map_transform_from_doc(doc):
    """
    Rebuild a MapTransform from a GraphQL `transform` document. Returns None when
    the document holds no usable transform (e.g. the server's placeholder).
    """
    if not doc:
        return None
    if doc.get('model') == 'tps':
        src = _unflat_points(doc.get('src_points') or [])
        dst = _unflat_points(doc.get('dst_points') or [])
        if len(src) == len(dst) and len(src) >= 3:
            return TpsMapTransform(src, dst, doc.get('smooth') or 0.0)
    R = doc.get('R') or []
    t = doc.get('t') or []
    if len(R) == 4 and len(t) == 2:
        return RigidMapTransform(R, t)
    return None
