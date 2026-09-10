"""ROS-independent mission decisions, geometry, and atomic exports."""
import math
import os
from pathlib import Path
import statistics
import yaml


class MissionClock:
    def __init__(self, duration=0.0):
        if not math.isfinite(duration) or duration < 0:
            raise ValueError('mission_duration_sec must be finite and >= 0')
        self.duration = duration
        self.started = None

    def elapsed(self, now):
        return 0.0 if self.started is None else max(0.0, now - self.started)

    def remaining(self, now):
        return None if self.duration == 0 else self.duration - self.elapsed(now)

    def fits(self, now, seconds):
        remaining = self.remaining(now)
        return remaining is None or remaining > seconds


def path_length(path):
    return sum(math.hypot(b.pose.position.x - a.pose.position.x,
                          b.pose.position.y - a.pose.position.y)
               for a, b in zip(path.poses, path.poses[1:]))


def cell(grid, x, y):
    q = grid.info.origin.orientation
    yaw = math.atan2(2 * (q.w*q.z + q.x*q.y), 1 - 2*(q.y*q.y + q.z*q.z))
    dx, dy = x-grid.info.origin.position.x, y-grid.info.origin.position.y
    c = math.floor((math.cos(yaw)*dx + math.sin(yaw)*dy) / grid.info.resolution)
    r = math.floor((-math.sin(yaw)*dx + math.cos(yaw)*dy) / grid.info.resolution)
    return c, r


def clear_point(grid, x, y, radius):
    """Require a circular footprint to lie entirely in observed free cells."""
    c, r = cell(grid, x, y)
    n = math.ceil(radius / grid.info.resolution)
    for dr in range(-n, n+1):
        for dc in range(-n, n+1):
            if math.hypot(dc, dr) > n:
                continue
            cc, rr = c+dc, r+dr
            if not (0 <= cc < grid.info.width and 0 <= rr < grid.info.height):
                return False
            value = grid.data[rr*grid.info.width+cc]
            if value < 0 or value >= 50:
                return False
    return True


def path_is_known(grid, path, radius):
    if not path or not path.poses:
        return False
    # Sample segments as well as vertices; sparse planner output must not skip walls.
    points = [p.pose.position for p in path.poses]
    for a, b in zip(points, points[1:] + points[-1:]):
        steps = max(1, math.ceil(math.hypot(b.x-a.x, b.y-a.y) / grid.info.resolution))
        for i in range(steps+1):
            t = i / steps
            if not clear_point(grid, a.x+t*(b.x-a.x), a.y+t*(b.y-a.y), radius):
                return False
    return True


def summarize(observations, threshold):
    positions = [o['position'] for o in observations]
    center = [statistics.median(p[i] for p in positions) for i in range(3)]
    inliers = [p for p in positions if math.dist(p, center) <= threshold]
    if inliers:
        center = [statistics.median(p[i] for p in inliers) for i in range(3)]
    return {'position': dict(zip(('x', 'y', 'z'), center)),
            'observations': len(positions), 'consistent_observations': len(inliers),
            'confirmed': len(inliers) >= 3,
            'spread_m': max((math.dist(p, center) for p in inliers), default=0.0)}


def atomic_yaml(path, data):
    path = Path(path).expanduser()
    path.parent.mkdir(parents=True, exist_ok=True)
    temporary = path.with_suffix(path.suffix + '.tmp')
    with temporary.open('w') as stream:
        yaml.safe_dump(data, stream, sort_keys=False)
        stream.flush()
        os.fsync(stream.fileno())
    temporary.replace(path)


def return_budget(path, speed=0.10, turn_speed=0.4, initial_yaw=0.0,
                  planning=10.0, recovery=20.0, margin=30.0):
    """Conservative route estimate with headings sampled at 25 cm intervals."""
    length = path_length(path)
    points = [p.pose.position for p in path.poses]
    headings = []
    if points:
        anchor = points[0]
        for point in points[1:]:
            if math.hypot(point.x-anchor.x, point.y-anchor.y) >= 0.25:
                headings.append(math.atan2(point.y-anchor.y, point.x-anchor.x))
                anchor = point
        if math.hypot(points[-1].x-anchor.x, points[-1].y-anchor.y) > 1e-6:
            headings.append(math.atan2(points[-1].y-anchor.y, points[-1].x-anchor.x))
    turns = 0.0
    previous = initial_yaw
    for heading in headings:
        turns += abs(math.atan2(math.sin(heading-previous), math.cos(heading-previous)))
        previous = heading
    parts = dict(travel_sec=length/speed, turning_sec=turns/turn_speed,
                 planning_sec=planning, recovery_sec=recovery, margin_sec=margin)
    parts['total_sec'] = sum(parts.values())
    return parts
