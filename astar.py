import sys
import json
import heapq
import math


# -----------------------
# Grid class
# -----------------------

# Global rules knob (read-only inside helpers)
RULES = {}

class Grid:
    def __init__(self, boundary, layers, step):
        self.boundary = boundary
        self.layers = layers
        self.step = step
        xs = [p[0] for p in boundary]
        ys = [p[1] for p in boundary]
        self.minx = min(xs)
        self.maxx = max(xs)
        self.miny = min(ys)
        self.maxy = max(ys)
        self.nx = int(math.ceil((self.maxx - self.minx) / step)) + 1
        self.ny = int(math.ceil((self.maxy - self.miny) / step)) + 1
        # occupancy: occ[layer_index][ix][iy]
        self.occ = [[[False for _ in range(self.ny)] for _ in range(self.nx)] for _ in layers]
        self.pad_mask = [[False for _ in range(self.ny)] for _ in range(self.nx)]

    def in_bounds(self, ix, iy):
        return 0 <= ix < self.nx and 0 <= iy < self.ny

    def world_to_grid(self, x, y):
        return (int(round((x - self.minx) / self.step)),
                int(round((y - self.miny) / self.step)))

    def grid_to_world(self, ix, iy):
        return (self.minx + ix * self.step,
                self.miny + iy * self.step)

    def is_blocked(self, ix, iy, il):
        return self.occ[il][ix][iy]

    def set_block(self, ix, iy, il):
        self.occ[il][ix][iy] = True

    def clear_block(self, ix, iy, il):
        """Forcefully mark a grid cell as free (used temporarily for start/goal)."""
        if 0 <= ix < self.nx and 0 <= iy < self.ny:
            self.occ[il][ix][iy] = False


def _inside_rotated_rect(cx, cy, px, py, sx, sy, rot_deg):
    """Is (cx,cy) inside a rectangle of half-sizes (sx,sy) centered at (px,py)
       rotated by rot_deg (degrees, CCW)?"""
    theta = math.radians(rot_deg or 0.0)
    ct, st = math.cos(theta), math.sin(theta)
    dx, dy = cx - px, cy - py
    # rotate point by -theta into pad-local frame
    lx =  dx*ct + dy*st
    ly = -dx*st + dy*ct
    return (-sx <= lx <= sx) and (-sy <= ly <= sy)


def clear_pad_region(grid, pad, layer):
    shape = pad.get("shape", "circle")
    clearance = float(pad.get("clearance", 0.2))
    rot = float(pad.get("abs_rotation", pad.get("pad_rotation", 0.0)))  # <-- use abs rot

    if shape in ("roundrect", "rect"):
        hx = float(pad.get("size_x", 0.0)) / 2.0 + clearance
        hy = float(pad.get("size_y", 0.0)) / 2.0 + clearance
        # bounding square to limit the scan
        b = max(hx, hy)
        ix0, iy0 = grid.world_to_grid(pad["x"] - b, pad["y"] - b)
        ix1, iy1 = grid.world_to_grid(pad["x"] + b, pad["y"] + b)
        for ix in range(ix0, ix1 + 1):
            for iy in range(iy0, iy1 + 1):
                if not grid.in_bounds(ix, iy):
                    continue
                cx, cy = grid.grid_to_world(ix, iy)
                if _inside_rotated_rect(cx, cy, pad["x"], pad["y"], hx, hy, rot):
                    grid.clear_block(ix, iy, layer)
    else:
        # circle branch stays as you had it
        r = float(pad.get("radius", 0.2)) + clearance
        ix0, iy0 = grid.world_to_grid(pad["x"] - r, pad["y"] - r)
        ix1, iy1 = grid.world_to_grid(pad["x"] + r, pad["y"] + r)
        for ix in range(ix0, ix1 + 1):
            for iy in range(iy0, iy1 + 1):
                if not grid.in_bounds(ix, iy):
                    continue
                cx, cy = grid.grid_to_world(ix, iy)
                dx, dy = cx - pad["x"], cy - pad["y"]
                if dx*dx + dy*dy <= r*r:
                    grid.clear_block(ix, iy, layer)


def set_pad_region(grid, pad, layer):
    shape = pad.get("shape", "circle")
    clearance = float(pad.get("clearance", 0.2)) + float(RULES.get("pad_clearance_extra", 0.0))
    rot = float(pad.get("abs_rotation", pad.get("pad_rotation", 0.0)))

    if shape in ("roundrect", "rect"):
        hx = float(pad.get("size_x", 0.0)) / 2.0 + clearance
        hy = float(pad.get("size_y", 0.0)) / 2.0 + clearance
        b = max(hx, hy)
        ix0, iy0 = grid.world_to_grid(pad["x"] - b, pad["y"] - b)
        ix1, iy1 = grid.world_to_grid(pad["x"] + b, pad["y"] + b)
        for ix in range(ix0, ix1 + 1):
            for iy in range(iy0, iy1 + 1):
                if not grid.in_bounds(ix, iy):
                    continue
                cx, cy = grid.grid_to_world(ix, iy)
                if _inside_rotated_rect(cx, cy, pad["x"], pad["y"], hx, hy, rot):
                    grid.set_block(ix, iy, layer)
    else:
        r = float(pad.get("radius", 0.2)) + clearance
        ix0, iy0 = grid.world_to_grid(pad["x"] - r, pad["y"] - r)
        ix1, iy1 = grid.world_to_grid(pad["x"] + r, pad["y"] + r)
        for ix in range(ix0, ix1 + 1):
            for iy in range(iy0, iy1 + 1):
                if not grid.in_bounds(ix, iy):
                    continue
                cx, cy = grid.grid_to_world(ix, iy)
                dx, dy = cx - pad["x"], cy - pad["y"]
                if dx*dx + dy*dy <= r*r:
                    grid.set_block(ix, iy, layer)



# -----------------------
# Helper functions
# -----------------------

def _normalize_layer_name(name, grid_layers, rules_net=None):
    if name in grid_layers:
        return name
    # treat wildcards or combined tokens as “pick something workable”
    allowed = (rules_net or {}).get("allowed_layers") or grid_layers
    # Prefer F.Cu, then B.Cu, else the first allowed
    for pref in ("F.Cu", "B.Cu"):
        if pref in allowed:
            return pref
    return allowed[0]


def rules_for_net(global_rules, net_name, net_id):
    """Return (merged_rules, class_name) for this net."""
    classes = global_rules.get("netclasses", {}) or {}
    mapping = global_rules.get("net_to_class", {}) or {}
    cls_name = mapping.get(net_name) or mapping.get(str(net_id)) or global_rules.get("default_netclass", "Default")
    overrides = classes.get(cls_name, {}) or {}

    # start from base rules but drop the big maps to avoid re-injecting them
    base = {k: v for k, v in global_rules.items() if k not in ("netclasses","net_to_class","default_netclass")}
    merged = {**base, **overrides}
    return merged, cls_name


def _block_disk(grid, ix, iy, il, radius_mm):
    """Block a disk of given radius (mm) around (ix,iy,layer)."""
    r_cells = int(math.ceil(radius_mm / grid.step))
    for dx in range(-r_cells, r_cells + 1):
        for dy in range(-r_cells, r_cells + 1):
            jx, jy = ix + dx, iy + dy
            if not grid.in_bounds(jx, jy):
                continue
            # distance in mm using grid.step
            if (dx * grid.step) ** 2 + (dy * grid.step) ** 2 <= radius_mm ** 2 + 1e-12:
                grid.set_block(jx, jy, il)


def block_path_as_obstacles(grid, rules, path_cells, vias_raw):
    """
    After a net is routed, mark its copper (trace & vias) as obstacles for later nets.
    Uses trace_width/2 + clearance as the blocking radius (plus optional extra).
    """
    trace_w = float(rules.get("trace_width", 0.25))
    clearance = float(rules.get("clearance", 0.2))
    extra = float(rules.get("trace_clearance_extra", 0.0))  # optional knob
    r_trace = 0.5 * trace_w + clearance + extra

    # Block each path cell on its layer (this already covers the centerline).
    for ix, iy, il in path_cells:
        _block_disk(grid, ix, iy, il, r_trace)

    # Block vias on both layers they connect (use via_size/2 + clearance).
    via_size = float(rules.get("via_size", 0.6))
    r_via = 0.5 * via_size + clearance + extra
    for v in vias_raw:
        vix, viy = grid.world_to_grid(v["x"], v["y"])
        for lname in (v["from"], v["to"]):
            il = grid.layers.index(lname)
            _block_disk(grid, vix, viy, il, r_via)


def find_pad_for_point(obstacles, x, y, layer_name, tol=0.8):
    """
    Return the pad (from obstacles) that corresponds to (x,y,layer_name).
    We use a simple L1 distance and a tolerance in mm.
    """
    best = None
    bestd = 1e9
    for o in obstacles:
        if o.get("type") != "pad":
            continue
        # If the pad is layer-specific, match it; otherwise it applies to all layers
        if o.get("layer") and o.get("layer") != layer_name:
            continue
        d = abs(float(o["x"]) - float(x)) + abs(float(o["y"]) - float(y))
        if d < bestd and d <= tol:
            best = o
            bestd = d
    return best


def clear_full_pad_access(grid, pad, layer_index, rules):
    """
    Temporarily clear the *entire* pad copper + a small extra so a track fits through.
    We do this only on the pad's routing layer (layer_index). After routing,
    call set_pad_region(grid, pad, layer_index) to restore blocking.
    """
    if not pad:
        return  # nothing to do

    # Extra opening so the trace (width) can pass the pad boundary comfortably
    trace_w = float(rules.get("trace_width", 0.25))
    extra = float(rules.get("pad_access_extra", 0.5 * trace_w))

    # Use a copy so we don't mutate the original pad dict
    pad_copy = dict(pad)
    base_clear = float(pad.get("clearance", float(rules.get("clearance", 0.2))))
    pad_copy["clearance"] = base_clear + extra

    # Clear full pad geometry (rect/roundrect/circle) on this layer
    clear_pad_region(grid, pad_copy, layer_index)


def compute_coords_extent(obstacles, tasks):
    xs, ys = [], []
    for obs in obstacles:
        if obs.get("type") == "pad":
            xs.append(obs["x"])
            ys.append(obs["y"])
        else:
            for x, y in obs.get("polygon", []):
                xs.append(x)
                ys.append(y)
    for task in tasks:
        if "start" in task:
            xs.append(task["start"]["x"])
            ys.append(task["start"]["y"])
        if "goal" in task:
            xs.append(task["goal"]["x"])
            ys.append(task["goal"]["y"])
    if not xs:
        return 0, 0, 0, 0, []
    return min(xs), max(xs), min(ys), max(ys), list(zip(xs, ys))


def expand_boundary_to_include(boundary, minx, maxx, miny, maxy, margin):
    if not boundary:
        return [(minx - margin, miny - margin),
                (maxx + margin, miny - margin),
                (maxx + margin, maxy + margin),
                (minx - margin, maxy + margin)]
    return boundary


def rasterize_obstacles(grid, rules, obstacles):
    clearance_default = float(rules.get("clearance", 0.2))

    for obs in obstacles:
        if obs.get("type") == "pad":
            x, y = obs["x"], obs["y"]
            pad_extra = float(rules.get("pad_clearance_extra", 0.0))
            clearance = float(obs.get("clearance", clearance_default)) + pad_extra
            shape = obs.get("shape", "circle")

            layer_name = obs.get("layer")
            layers = [layer_name] if layer_name else grid.layers

            if shape in ("roundrect", "rect"):
                sx = float(obs.get("size_x", 0.0)) / 2.0 + clearance
                sy = float(obs.get("size_y", 0.0)) / 2.0 + clearance
                ix0, iy0 = grid.world_to_grid(x - sx, y - sy)
                ix1, iy1 = grid.world_to_grid(x + sx, y + sy)
                for il, lname in enumerate(grid.layers):
                    if lname not in layers:
                        continue
                    for ix in range(ix0, ix1 + 1):
                        for iy in range(iy0, iy1 + 1):
                            if not grid.in_bounds(ix, iy):
                                continue
                            cx, cy = grid.grid_to_world(ix, iy)
                            if (x - sx) <= cx <= (x + sx) and (y - sy) <= cy <= (y + sy):
                                grid.pad_mask[ix][iy] = True
                                grid.set_block(ix, iy, il)

            elif shape == "circle":
                r = float(obs.get("radius", 0.2))
                total_r = r + clearance
                ix0, iy0 = grid.world_to_grid(x - total_r, y - total_r)
                ix1, iy1 = grid.world_to_grid(x + total_r, y + total_r)
                for il, lname in enumerate(grid.layers):
                    if lname not in layers:
                        continue
                    for ix in range(ix0, ix1 + 1):
                        for iy in range(iy0, iy1 + 1):
                            if not grid.in_bounds(ix, iy):
                                continue
                            cx, cy = grid.grid_to_world(ix, iy)
                            dx, dy = cx - x, cy - y
                            if dx * dx + dy * dy <= (total_r * total_r):
                                grid.pad_mask[ix][iy] = True
                                grid.set_block(ix, iy, il)


        elif obs.get("polygon"):
            poly = obs["polygon"]
            xs = [px for px, _ in poly]
            ys = [py for _, py in poly]
            minx, maxx = min(xs), max(xs)
            miny, maxy = min(ys), max(ys)
            clearance = float(obs.get("clearance", clearance_default))
            grow = int(math.ceil(clearance / grid.step))
            ix0, iy0 = grid.world_to_grid(minx, miny)
            ix1, iy1 = grid.world_to_grid(maxx, maxy)
            layers = obs.get("layers", grid.layers)
            for il, lname in enumerate(grid.layers):
                if lname not in layers:
                    continue
                for ix in range(ix0 - grow, ix1 + grow + 1):
                    for iy in range(iy0 - grow, iy1 + grow + 1):
                        if grid.in_bounds(ix, iy):
                            grid.set_block(ix, iy, il)


def octile(x0, y0, x1, y1):
    dx = abs(x1 - x0)
    dy = abs(y1 - y0)
    return (dx + dy) + (math.sqrt(2) - 2) * min(dx, dy)


def compress_collinear(path):
    if not path:
        return []
    out = [path[0]]
    for i in range(1, len(path) - 1):
        x0, y0, l0 = out[-1]
        x1, y1, l1 = path[i]
        x2, y2, l2 = path[i + 1]
        dx1, dy1 = x1 - x0, y1 - y0
        dx2, dy2 = x2 - x1, y2 - y1
        if l0 == l1 == l2 and (dx1, dy1) != (0, 0):
            if dx1 * dy2 == dy1 * dx2:
                continue
        out.append((x1, y1, l1))
    out.append(path[-1])
    return out


# -----------------------
# A* search helper (balanced via rules)
# -----------------------

def astar_search(grid, rules, start_state, goal_state, allow_via):
    """Generic A* search with balanced via rules + min distance from pads for vias."""
    start_ix, start_iy, start_layer = start_state
    goal_ix, goal_iy, goal_layer = goal_state

    # Heuristics / costs
    via_cost = float(rules.get("via_cost", 10.0))  # balanced penalty

    # NEW: min distance (mm) from start/goal pad centers required to drop a via
    min_via_from_pads = float(rules.get("min_via_from_pads", 0.6))
    sx, sy = grid.grid_to_world(start_ix, start_iy)
    gx, gy = grid.grid_to_world(goal_ix, goal_iy)

    neighbors = [(1, 0, 1.0), (-1, 0, 1.0), (0, 1, 1.0), (0, -1, 1.0),
                 (1, 1, math.sqrt(2)), (-1, -1, math.sqrt(2)),
                 (1, -1, math.sqrt(2)), (-1, 1, math.sqrt(2))]
    
    allowed_layers = rules.get("allowed_layers")
    allowed_via_layers = None
    if allowed_layers:
        allowed_via_layers = {i for i, name in enumerate(grid.layers) if name in allowed_layers}

    open_heap = []
    start_state_key = (start_ix, start_iy, start_layer)
    goal_state_key = (goal_ix, goal_iy, goal_layer)
    g_score = {start_state_key: 0.0}
    parent = {start_state_key: None}

    def heuristic(ix, iy, il):
        h = octile(ix, iy, goal_ix, goal_iy)
        if il != goal_layer:
            h += via_cost
        return h

    def push_state(state, gval):
        f = gval + heuristic(state[0], state[1], state[2])
        heapq.heappush(open_heap, (f, state))

    push_state(start_state_key, 0.0)

    EPS = 1e-9

    while open_heap:
        f, cur = heapq.heappop(open_heap)
        cur_g = g_score.get(cur, float('inf'))
        if cur_g + heuristic(cur[0], cur[1], cur[2]) + EPS < f:
            continue

        if cur == goal_state_key:
            path = []
            vias = []
            node = cur
            while node is not None:
                path.append(node)
                node = parent.get(node)
            path.reverse()
            for i in range(1, len(path)):
                if path[i][2] != path[i - 1][2]:
                    x, y = grid.grid_to_world(path[i][0], path[i][1])
                    vias.append({
                        "x": x,
                        "y": y,
                        "from": grid.layers[path[i - 1][2]],
                        "to": grid.layers[path[i][2]]
                    })
            return path, vias

        ix, iy, il = cur
        gcur = g_score.get(cur, float('inf'))

        # ---- planar neighbors on same layer ----
        best_neighbor_heuristic = float('inf')
        planar_moves_exist = False
        for dx, dy, dcost in neighbors:
            jx, jy = ix + dx, iy + dy
            if not grid.in_bounds(jx, jy):
                continue
            if grid.is_blocked(jx, jy, il):
                continue
            planar_moves_exist = True
            h_neighbor = octile(jx, jy, goal_ix, goal_iy)
            if h_neighbor < best_neighbor_heuristic:
                best_neighbor_heuristic = h_neighbor
            cand = (jx, jy, il)
            tg = gcur + dcost
            if tg + EPS < g_score.get(cand, float('inf')):
                parent[cand] = cur
                g_score[cand] = tg
                push_state(cand, tg)

        # ---- via policy (original gating) ----
        allow_via_here = False
        if not planar_moves_exist:
            allow_via_here = True
        else:
            current_h = octile(ix, iy, goal_ix, goal_iy)
            if best_neighbor_heuristic >= current_h - 1e-9:
                allow_via_here = True

        # ---- propose via (with new min-distance-from-pads rule) ----
        if allow_via and allow_via_here:
            cx, cy = grid.grid_to_world(ix, iy)

            # must be at least D mm from BOTH start and goal pad centers
            if (math.hypot(cx - sx, cy - sy) >= min_via_from_pads and
                math.hypot(cx - gx, cy - gy) >= min_via_from_pads):

                for jl in range(len(grid.layers)):
                    if jl == il:
                        continue
                    if allowed_via_layers is not None and jl not in allowed_via_layers:
                        continue
                    if grid.is_blocked(ix, iy, jl):
                        continue
                    if grid.is_blocked(ix, iy, il):
                        continue

                    cand = (ix, iy, jl)
                    tg = gcur + via_cost
                    if tg + EPS < g_score.get(cand, float('inf')):
                        parent[cand] = cur
                        g_score[cand] = tg
                        push_state(cand, tg)

    raise RuntimeError("No route found")



def astar_route(grid, rules, start, goal):
    if start is None or goal is None:
        raise RuntimeError("Missing start or goal in astar_route")

    start_ix, start_iy = grid.world_to_grid(start["x"], start["y"])
    goal_ix, goal_iy = grid.world_to_grid(goal["x"], goal["y"])

    if not grid.in_bounds(start_ix, start_iy) or not grid.in_bounds(goal_ix, goal_iy):
        raise RuntimeError("Start or goal out of grid bounds in astar_route")

    start_layer = grid.layers.index(start.get("layer", grid.layers[0]))
    goal_layer = grid.layers.index(goal.get("layer", grid.layers[0]))
    start_state = (start_ix, start_iy, start_layer)
    goal_state = (goal_ix, goal_iy, goal_layer)

    if start_layer == goal_layer:
        try:
            path, vias = astar_search(grid, rules, start_state, goal_state, allow_via=False)
            return path, vias
        except RuntimeError:
            pass

    path, vias = astar_search(grid, rules, start_state, goal_state, allow_via=True)
    return path, vias


# -----------------------
# Geometry helpers for DRC (left in place but no longer used)
# -----------------------

def _local_to_world(lx, ly, px, py, rot_deg):
    t = math.radians(rot_deg or 0.0)
    ct, st = math.cos(t), math.sin(t)
    # rotate by +theta and shift back to (px,py)
    wx = px + lx*ct - ly*st
    wy = py + lx*st + ly*ct
    return wx, wy

def _dot(ax, ay, bx, by):
    return ax*bx + ay*by

def _clamp01(t):
    return 0.0 if t < 0.0 else (1.0 if t > 1.0 else t)

def _seg_seg_distance(p1, p2, q1, q2):
    """
    Return (dist, cp1, cp2, intersects_centerlines) between segments p1->p2 and q1->q2.
    cp1/cp2 are closest points (x,y).
    """
    x1,y1 = p1; x2,y2 = p2
    x3,y3 = q1; x4,y4 = q2
    ux, uy = x2-x1, y2-y1
    vx, vy = x4-x3, y4-y3
    wx, wy = x1-x3, y1-y3
    a = _dot(ux,uy,ux,uy)       # |u|^2
    b = _dot(ux,uy,vx,vy)
    c = _dot(vx,vy,vx,vy)       # |v|^2
    d = _dot(ux,uy,wx,wy)
    e = _dot(vx,vy,wx,wy)
    D = a*c - b*b
    sc, sN, sD = 0.0, D, D
    tc, tN, tD = 0.0, D, D

    if D < 1e-12:
        sN = 0.0
        sD = 1.0
        tN = e
        tD = c
    else:
        sN = (b*e - c*d)
        tN = (a*e - b*d)
        if sN < 0.0:
            sN = 0.0
            tN = e
            tD = c
        elif sN > sD:
            sN = sD
            tN = e + b
            tD = c

    if tN < 0.0:
        tN = 0.0
        if -d < 0.0:
            sN = 0.0
        elif -d > a:
            sN = sD
        else:
            sN = -d
            sD = a
    elif tN > tD:
        tN = tD
        if (-d + b) < 0.0:
            sN = 0.0
        elif (-d + b) > a:
            sN = sD
        else:
            sN = (-d + b)
            sD = a

    sc = 0.0 if abs(sN) < 1e-12 else (sN / sD)
    tc = 0.0 if abs(tN) < 1e-12 else (tN / tD)

    cx1 = x1 + sc*ux; cy1 = y1 + sc*uy
    cx2 = x3 + tc*vx; cy2 = y3 + tc*vy
    dx = cx1 - cx2; dy = cy1 - cy2
    dist = math.hypot(dx, dy)

    # Exact centerline intersection?
    # If segments strictly cross, both sc and tc in (0,1) and dist ~ 0
    intersects = (dist < 1e-6) and (0.0 <= sc <= 1.0) and (0.0 <= tc <= 1.0)
    return dist, (cx1, cy1), (cx2, cy2), intersects

def _pt_seg_distance(pt, a, b):
    """Distance from point to segment."""
    x,y = pt; x1,y1 = a; x2,y2 = b
    vx,vy = x2-x1,y2-y1
    if abs(vx) < 1e-12 and abs(vy) < 1e-12:
        return math.hypot(x-x1,y-y1), (x1,y1)
    t = _clamp01(((x-x1)*vx + (y-y1)*vy) / (vx*vx + vy*vy))
    cx,cy = x1 + t*vx, y1 + t*vy
    return math.hypot(x-cx, y-cy), (cx,cy)

def _dist_seg_to_circle(seg_a, seg_b, cx, cy, radius):
    """Centerline distance from segment to circle boundary (>=0 outside, <0 overlap)."""
    d, cp = _pt_seg_distance((cx,cy), seg_a, seg_b)
    return d - radius, cp

def _rotate_to_local(x, y, px, py, rot_deg):
    """Rotate (x,y) around (px,py) by -rot_deg to pad-local frame."""
    t = math.radians(rot_deg or 0.0)
    ct, st = math.cos(t), math.sin(t)
    dx, dy = x - px, y - py
    lx =  dx*ct + dy*st
    ly = -dx*st + dy*ct
    return lx, ly

def _seg_rect_distance_local(a, b, hx, hy):
    """
    Distance between segment AB and axis-aligned rectangle centered at origin with half sizes hx, hy.
    Returns (dist, cp_on_seg) with dist >= 0 when outside, < 0 when overlapping.
    """
    # If either endpoint inside rectangle, overlap
    for px,py in (a,b):
        if (-hx <= px <= hx) and (-hy <= py <= hy):
            # measure how deep inside (negative clearance as min distance to an edge)
            d = -min(hx - abs(px), hy - abs(py))
            return d, (px,py)

    # Rectangle edges as segments
    edges = [
        ((-hx,-hy), ( hx,-hy)),
        (( hx,-hy), ( hx, hy)),
        (( hx, hy), (-hx, hy)),
        ((-hx, hy), (-hx,-hy)),
    ]

    best_d = 1e9
    best_cp = None
    for e1, e2 in edges:
        d, cp1, cp2, _ = _seg_seg_distance(a, b, e1, e2)
        if d < best_d:
            best_d = d
            best_cp = cp1
    return best_d, best_cp

def _track_pad_clearance(seg_a, seg_b, width, pad):
    """
    Signed clearance from a track (centerline AB, width) to a pad (rect/roundrect/circle).
    Positive => clearance; negative => overlap.
    Returns (actual_clearance, at_point).
    """
    shape = pad.get("shape","rect")
    rot = float(pad.get("abs_rotation", pad.get("pad_rotation", 0.0)))
    px, py = pad["x"], pad["y"]

    if shape in ("rect","roundrect"):
        hx = float(pad.get("size_x",0.0))/2.0
        hy = float(pad.get("size_y",0.0))/2.0
        # Transform segment endpoints to pad local frame
        a_loc = _rotate_to_local(seg_a[0], seg_a[1], px, py, rot)
        b_loc = _rotate_to_local(seg_b[0], seg_b[1], px, py, rot)
        d, cp = _seg_rect_distance_local(a_loc, b_loc, hx, hy)
        # Clearance from track edge to pad edge:
        cp_world = _local_to_world(cp[0], cp[1], px, py, rot)
        return d - (width/2.0), cp_world
    else:
        # treat as circle
        r = float(pad.get("radius", 0.2))
        d, cp = _dist_seg_to_circle(seg_a, seg_b, px, py, r)
        cp_world = _local_to_world(cp[0], cp[1], px, py, rot)
        return d - (width/2.0), cp_world

def _via_pad_clearance(cx, cy, via_diam, pad):
    """Signed clearance between via (circle) and pad."""
    shape = pad.get("shape","rect")
    rot = float(pad.get("abs_rotation", pad.get("pad_rotation", 0.0)))
    px, py = pad["x"], pad["y"]

    if shape in ("rect","roundrect"):
        hx = float(pad.get("size_x",0.0))/2.0
        hy = float(pad.get("size_y",0.0))/2.0
        v_loc = _rotate_to_local(cx, cy, px, py, rot)
        # distance from a POINT to rectangle (>=0 outside, <0 inside)
        x, y = v_loc
        dx = max(abs(x) - hx, 0.0)
        dy = max(abs(y) - hy, 0.0)
        if dx == 0.0 and dy == 0.0:
            dist = -min(hx - abs(x), hy - abs(y))
        else:
            dist = math.hypot(dx, dy)
        return dist - (via_diam/2.0), (cx,cy)
    else:
        r = float(pad.get("radius",0.2))
        dist = math.hypot(cx - px, cy - py) - r
        return dist - (via_diam/2.0), (cx,cy)


# -----------------------
# (Old) DRC Engine (kept, but unused)
# -----------------------

def _obj_desc_track(net, layer, a, b):
    return f'Track [{net}] on {layer}, ({a[0]:.4f},{a[1]:.4f})→({b[0]:.4f},{b[1]:.4f})'

def _obj_desc_via(net, cx, cy, d):
    return f'Via [{net}] Ø{d:.3f} at ({cx:.4f},{cy:.4f})'

def _obj_desc_pad(pad):
    return f'Pad [{pad.get("net")}] {pad.get("shape","rect")} at ({pad["x"]:.4f},{pad["y"]:.4f})'

def run_drc(board, rules_global, obstacles, routes):
    # Left here in case you want to re-enable later.
    return []


# -----------------------
# Routing manager
# -----------------------

def route_all(input_data):
    board = input_data.get("board", {})
    rules = input_data.get("rules", {})
    # make rules available to helpers
    global RULES
    RULES = rules
    obstacles = input_data.get("obstacles", [])
    tasks = input_data.get("tasks", [])

    boundary = board.get("boundary", []) or []
    layers = board.get("layers", [])
    step = float(rules.get("grid_step", 0.1))

    minx, maxx, miny, maxy, coords = compute_coords_extent(obstacles, tasks)
    margin = max(step * 10, 5.0)
    boundary_expanded = expand_boundary_to_include(boundary, minx, maxx, miny, maxy, margin)

    grid = Grid(boundary_expanded, layers, step)
    rasterize_obstacles(grid, rules, obstacles)

    routes = []
    for task in tasks:
        net = task.get("net")
        net_id = task.get("net_id")
        start = task.get("start")
        goal = task.get("goal")
        if not start or not goal:
            routes.append({"net": net, "net_id": net_id, "failed": True, "reason": "Missing start or goal"})
            continue
        rules_net, cls_name = rules_for_net(rules, net, net_id)
        s_layer_name = _normalize_layer_name(start.get("layer", grid.layers[0]), grid.layers, rules_net)
        g_layer_name = _normalize_layer_name(goal.get("layer",  grid.layers[0]), grid.layers, rules_net)
        start_layer  = grid.layers.index(s_layer_name)
        goal_layer   = grid.layers.index(g_layer_name)

        # Find real pads so we can clear the full pad geometry (not just the center dot)
        start_pad = find_pad_for_point(obstacles, start["x"], start["y"], grid.layers[start_layer])
        goal_pad  = find_pad_for_point(obstacles, goal["x"],  goal["y"],  grid.layers[goal_layer])

        # Open full pad copper (plus a little extra) so a trace can enter/exit
        clear_full_pad_access(grid, start_pad, start_layer, rules_net)
        clear_full_pad_access(grid, goal_pad,  goal_layer,  rules_net)

        try:
            path_cells, vias_raw = astar_route(grid, rules_net, start, goal)
        except Exception as e:
            # Restore original pad blocking before reporting failure
            if start_pad: set_pad_region(grid, start_pad, start_layer)
            if goal_pad:  set_pad_region(grid, goal_pad,  goal_layer)
            routes.append({"net": net, "net_id": net_id, "failed": True, "reason": str(e), "netclass": cls_name})
            continue

        # Restore original pad blocking after success
        if start_pad: set_pad_region(grid, start_pad, start_layer)
        if goal_pad:  set_pad_region(grid, goal_pad,  goal_layer)
 
        # lock in this route as an obstacle so later nets can't cross it
        block_path_as_obstacles(grid, rules_net, path_cells, vias_raw)

        # Convert path to world segments
        world_path = []
        for ix, iy, il in path_cells:
            x, y = grid.grid_to_world(ix, iy)
            world_path.append((round(x, 6), round(y, 6), grid.layers[il]))

        simplified = compress_collinear(world_path)

        segments = []
        for i in range(len(simplified) - 1):
            x0, y0, l0 = simplified[i]
            x1, y1, l1 = simplified[i + 1]
            if l0 == l1:
                segments.append({
                    "start": [x0, y0],
                    "end": [x1, y1],
                    "layer": l0,
                    "width": float(rules_net.get("trace_width", 0.25))
                })

        vias = []
        for via in vias_raw:
            vias.append({
                "at": [round(via["x"], 6), round(via["y"], 6)],
                "from": via["from"],
                "to": via["to"],
                "size": float(rules_net.get("via_size", 0.6)),
                "drill": float(rules_net.get("via_drill", 0.3))
            })

        routes.append({
            "net": net,
            "net_id": net_id,
            "netclass": cls_name,
            "segments": segments,
            "vias": vias
        })

    # ----------- DRC stage removed -----------
    return {"routes": routes}


# -----------------------
# CLI
# -----------------------

def main():
    if len(sys.argv) < 2:
        print("Usage: python astar.py input.json [output.json]")
        sys.exit(1)
    with open(sys.argv[1], "r") as f:
        data = json.load(f)
    out = route_all(data)
    if len(sys.argv) >= 3:
        with open(sys.argv[2], "w") as f:
            json.dump(out, f, indent=2)
        print(f"Routing finished. Results written to {sys.argv[2]}")
    else:
        print(json.dumps(out, indent=2))


if __name__ == "__main__":
    main()
