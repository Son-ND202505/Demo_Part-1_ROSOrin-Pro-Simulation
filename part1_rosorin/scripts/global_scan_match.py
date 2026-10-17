"""Coarse-to-fine 2D scan-to-map initialization; no simulator pose input."""
import math
import time
import numpy as np
from scipy.ndimage import map_coordinates
from scipy.optimize import minimize


def find_pose(grid, distances, resolution, origin, scan, laser_pose, deadline):
    """Return a distinct scan-supported base pose, or None if ambiguous/weak.

    x/y search spacing ~0.4 m, heading spacing 10 degrees, then local refinement.
    Unknown cells and out-of-map endpoints do not count as obstacle matches.
    This heuristic cannot disambiguate geometrically identical rooms.
    """
    ox, oy, oa = origin
    lx, ly, la = laser_pose
    ranges = np.asarray(scan.ranges)
    valid = np.flatnonzero(np.isfinite(ranges) & (ranges > scan.range_min)
                           & (ranges < scan.range_max))
    if len(valid) < 30:
        return None
    valid = valid[::max(1, len(valid)//60)]
    ranges = ranges[valid]
    angles = scan.angle_min + valid * scan.angle_increment + la
    bx = lx + ranges * np.cos(angles)
    by = ly + ranges * np.sin(angles)
    cos_o, sin_o = math.cos(oa), math.sin(oa)
    float_grid = grid.astype(float)

    def scores(poses):
        poses = np.atleast_2d(poses)
        cs, sn = np.cos(poses[:, 2:3]), np.sin(poses[:, 2:3])
        def cells(xb, yb):
            x = poses[:, 0:1] + cs*xb - sn*yb - ox
            y = poses[:, 1:2] + sn*xb + cs*yb - oy
            return ((-sin_o*x + cos_o*y)/resolution - .5,
                    (cos_o*x + sin_o*y)/resolution - .5)
        rows, cols = cells(bx, by)
        d = map_coordinates(distances, [rows, cols], order=1, mode='constant', cval=5.)
        end = np.exp(-.5 * (d/.12)**2).mean(axis=1)
        free = np.zeros(len(poses))
        for fraction in (.25, .5, .75):
            r, c = cells(lx + fraction*(bx-lx), ly + fraction*(by-ly))
            cells_at = map_coordinates(float_grid, [r, c], order=0,
                                      mode='constant', cval=100.)
            free += (cells_at == 0).mean(axis=1)/3
        r, c = cells(np.array([0.]), np.array([0.]))
        centre = map_coordinates(float_grid, [r, c], order=0, mode='constant', cval=100.)[:, 0]
        return np.where(centre == 0, .8*end + .2*free, -1.)

    stride = max(1, round(.4/resolution))
    rr, cc = np.mgrid[0:grid.shape[0]:stride, 0:grid.shape[1]:stride]
    mask = (grid[rr, cc] == 0) & (distances[rr, cc] >= .16)
    rr, cc = rr[mask], cc[mask]
    if not len(rr) or len(rr) > 20000:
        return None
    gx, gy = (cc+.5)*resolution, (rr+.5)*resolution
    x, y = ox + cos_o*gx - sin_o*gy, oy + sin_o*gx + cos_o*gy
    candidates, values = [], []
    for angle in np.arange(-math.pi, math.pi, math.pi/18):
        if time.monotonic() >= deadline:
            return None
        poses = np.column_stack([x, y, np.full_like(x, angle)])
        score = scores(poses)
        top = np.argsort(score)[-8:]
        candidates.extend(poses[top])
        values.extend(score[top])
    selected = []
    for idx in np.argsort(values)[::-1]:
        p = np.asarray(candidates[idx])
        if any(np.linalg.norm(p[:2]-q[:2]) < .5 and
               abs(math.atan2(math.sin(p[2]-q[2]), math.cos(p[2]-q[2]))) < .4 for q in selected):
            continue
        selected.append(p)
        if len(selected) >= 8:
            break
    refined = []
    for p in selected:
        if time.monotonic() >= deadline:
            return None
        fit = minimize(lambda z: -float(scores(z)[0]), p, method='Powell',
                       bounds=[(p[0]-.45, p[0]+.45), (p[1]-.45, p[1]+.45),
                               (p[2]-.22, p[2]+.22)], options={'maxiter': 30, 'xtol': .005})
        refined.append((float(-fit.fun), fit.x))
    refined.sort(key=lambda item: item[0], reverse=True)
    best, p = refined[0]
    alternate = max((s for s, q in refined[1:] if np.linalg.norm(p[:2]-q[:2]) > .6 or
                     abs(math.atan2(math.sin(p[2]-q[2]), math.cos(p[2]-q[2]))) > .5), default=0.)
    if best < .65 or best - alternate < .035:
        return None
    return {'x': float(p[0]), 'y': float(p[1]), 'yaw': float(p[2]),
            'score': best, 'alternative_score': alternate}
