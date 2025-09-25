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

    if shape in ("roundrect", "rect", "oval"):
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
        # circle branch
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

    if shape in ("roundrect", "rect", "oval"):
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
    # Treat KiCad "*.Cu" as "pick a practical copper layer"
    if name == "*.Cu":
        # Prefer F.Cu, then B.Cu, else first copper layer
        for pref in ("F.Cu", "B.Cu"):
            if pref in grid_layers:
                return pref
        return grid_layers[0]

    if name in grid_layers:
        return name

    allowed = (rules_net or {}).get("allowed_layers") or grid_layers
    for pref in ("F.Cu", "B.Cu"):
        if pref in allowed:
            return pref
    return allowed[0]


def _clear_disk(grid, ix, iy, il, radius_mm):
    """Temporarily clear a disk of given radius (mm) on (ix,iy,layer)."""
    r_cells = int(math.ceil(radius_mm / grid.step))
    for dx in range(-r_cells, r_cells + 1):
        for dy in range(-r_cells, r_cells + 1):
            jx, jy = ix + dx, iy + dy
            if not grid.in_bounds(jx, jy):
                continue
            if (dx * grid.step) ** 2 + (dy * grid.step) ** 2 <= radius_mm ** 2 + 1e-12:
                grid.clear_block(jx, jy, il)


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

def _disk_is_free(grid, ix, iy, il, radius_mm):
    """
    True iff every grid cell within 'radius_mm' of (ix,iy) on layer 'il'
    is free of obstacles/pad copper.
    """
    r_cells = int(math.ceil(radius_mm / grid.step))
    r2 = radius_mm * radius_mm + 1e-12
    for dx in range(-r_cells, r_cells + 1):
        for dy in range(-r_cells, r_cells + 1):
            if (dx * grid.step) ** 2 + (dy * grid.step) ** 2 > r2:
                continue
            jx, jy = ix + dx, iy + dy
            if not grid.in_bounds(jx, jy):
                return False
            if grid.occ[il][jx][jy] or grid.pad_mask[jx][jy]:
                return False
    return True


def block_path_as_obstacles(grid, rules, path_cells, vias_raw):
    """
    After a net is routed, mark its copper (trace & vias) as obstacles for later nets.
    Uses trace_width/2 + clearance as the blocking radius (plus optional extra).
    """
    trace_w   = float(rules.get("trace_width", 0.25))
    clearance = float(rules.get("clearance", 0.2))
    extra_t   = float(rules.get("trace_clearance_extra", 0.0))
    r_trace   = 0.5 * trace_w + clearance + extra_t

    for ix, iy, il in path_cells:
        _block_disk(grid, ix, iy, il, r_trace)

    via_size  = float(rules.get("via_size", 0.6))
    extra_v   = float(rules.get("via_clearance_extra", 0.0))  # optional knob
    r_via     = 0.5 * via_size + clearance + extra_v
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
        pad_layer = o.get("layer")
        # "*.Cu" matches any copper layer
        if pad_layer and pad_layer not in (layer_name, "*.Cu"):
            continue
        d = abs(float(o["x"]) - float(x)) + abs(float(o["y"]) - float(y))
        if d < bestd and d <= tol:
            best = o
            bestd = d
    return best


def clear_full_pad_access(grid, pad, layer_index, rules):
    """
    Temporarily clear the *entire* pad copper + a small extra so a track fits through.
    Do this on the pad's routing layer (layer_index). After routing,
    call set_pad_region(grid, pad, layer_index) to restore blocking.
    """
    if not pad:
        return

    # Extra opening so the trace (width) can pass the pad boundary comfortably
    trace_w = float(rules.get("trace_width", 0.25))
    extra = float(rules.get("pad_access_extra", 0.5 * trace_w))

    # Use a copy so we don't mutate the original pad dict
    pad_copy = dict(pad)
    base_clear = float(pad.get("clearance", float(rules.get("clearance", 0.2))))
    pad_copy["clearance"] = base_clear + extra

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
    """
    Rasterize pads (SMD + THT) and polygon obstacles into grid. 
    Rect/roundrect/oval pads use a rotated-rectangle test so rotation/orientation is honored.
    THT pads ("*.Cu") get an optional extra clearance ring: rules["tht_clearance_extra"].
    """
    clearance_default = float(rules.get("clearance", 0.2))
    pad_extra         = float(rules.get("pad_clearance_extra", 0.0))
    tht_extra         = float(rules.get("tht_clearance_extra", 0.0))  # <— NEW

    for obs in obstacles:
        if obs.get("type") == "pad":
            x = float(obs["x"]); y = float(obs["y"])
            shape = obs.get("shape", "circle")
            rot   = float(obs.get("abs_rotation", obs.get("pad_rotation", 0.0)))

            layer_name = obs.get("layer")
            if layer_name == "*.Cu":
                layers = grid.layers[:]                 # THT: block on all copper layers
            elif layer_name:
                layers = [layer_name]
            else:
                layers = grid.layers[:]

            # base + project pad bump (+ extra for THT)
            base_clear = float(obs.get("clearance", clearance_default))
            clearance  = base_clear + pad_extra + (tht_extra if layer_name == "*.Cu" else 0.0)

            if shape in ("roundrect", "rect", "oval"):
                # Treat oval as a rotated rectangle using size_x/size_y
                hx = float(obs.get("size_x", 0.0)) / 2.0 + clearance
                hy = float(obs.get("size_y", 0.0)) / 2.0 + clearance

                # scan a bounding box, test with rotated-rect predicate
                bb = max(hx, hy)
                ix0, iy0 = grid.world_to_grid(x - bb, y - bb)
                ix1, iy1 = grid.world_to_grid(x + bb, y + bb)

                for il, lname in enumerate(grid.layers):
                    if lname not in layers:
                        continue
                    for ix in range(ix0, ix1 + 1):
                        for iy in range(iy0, iy1 + 1):
                            if not grid.in_bounds(ix, iy):
                                continue
                            cx, cy = grid.grid_to_world(ix, iy)
                            if _inside_rotated_rect(cx, cy, x, y, hx, hy, rot):
                                grid.pad_mask[ix][iy] = True
                                grid.set_block(ix, iy, il)

            elif shape == "circle":
                r = float(obs.get("radius", 0.2)) + clearance
                ix0, iy0 = grid.world_to_grid(x - r, y - r)
                ix1, iy1 = grid.world_to_grid(x + r, y + r)

                for il, lname in enumerate(grid.layers):
                    if lname not in layers:
                        continue
                    for ix in range(ix0, ix1 + 1):
                        for iy in range(iy0, iy1 + 1):
                            if not grid.in_bounds(ix, iy):
                                continue
                            cx, cy = grid.grid_to_world(ix, iy)
                            dx, dy = cx - x, cy - y
                            if dx*dx + dy*dy <= r*r:
                                grid.pad_mask[ix][iy] = True
                                grid.set_block(ix, iy, il)

        elif obs.get("polygon"):
            # Simple polygon grow-by-clearance raster (unchanged)
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


def compress_collinear(path, ang_eps_deg=5.0):
    if not path:
        return []
    out = [path[0]]
    for i in range(1, len(path) - 1):
        x0, y0, l0 = out[-1]
        x1, y1, l1 = path[i]
        x2, y2, l2 = path[i + 1]
        dx1, dy1 = x1 - x0, y1 - y0
        dx2, dy2 = x2 - x1, y2 - y1
        if l0 == l1 == l2 and (dx1 or dy1) and (dx2 or dy2):
            if dx1 * dy2 == dy1 * dx2:
                continue
            a1 = math.atan2(dy1, dx1)
            a2 = math.atan2(dy2, dx2)
            da = (a2 - a1 + math.pi) % (2*math.pi) - math.pi
            if abs(da) < math.radians(ang_eps_deg):
                continue
        out.append((x1, y1, l1))
    out.append(path[-1])
    return out



# -----------------------
# A* search helper (balanced via rules)
# -----------------------

def astar_search(grid, rules, start_state, goal_state, allow_via, start_xy, goal_xy):
    """Generic A* search with balanced via rules + min distance from pads for vias."""
    start_ix, start_iy, start_layer = start_state
    goal_ix, goal_iy, goal_layer = goal_state

    # Heuristics / costs
    via_cost = float(rules.get("via_cost", 10.0))  # balanced penalty
    min_via_from_pads = float(rules.get("min_via_from_pads", 0.6))

    # Via radius used for local clearance check
    via_size  = float(rules.get("via_size", 0.6))
    clearance = float(rules.get("clearance", 0.2))
    extra_v   = float(rules.get("via_clearance_extra", 0.0))
    r_via     = 0.5 * via_size + clearance + extra_v

    sx, sy = float(start_xy[0]), float(start_xy[1])
    gx, gy = float(goal_xy[0]),  float(goal_xy[1])

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

        # ---- propose via (EARLY-WINDOW + stall gate + full-disk clearance) ----
        if allow_via:
            cx, cy = grid.grid_to_world(ix, iy)

            # Via allowed if either:
            #  1) we're far enough from the nearer pad center (early window), OR
            #  2) the search is "stalled" (no strictly better planar move)
            d_start = math.hypot(cx - sx, cy - sy)
            d_goal  = math.hypot(cx - gx, cy - gy)
            distance_ok = (min(d_start, d_goal) >= min_via_from_pads)

            allow_drop = allow_via_here or distance_ok

            if allow_drop:
                for jl in range(len(grid.layers)):
                    if jl == il:
                        continue
                    if allowed_via_layers is not None and jl not in allowed_via_layers:
                        continue
                    if grid.is_blocked(ix, iy, jl) or grid.is_blocked(ix, iy, il):
                        continue
                    # Ensure the *full* via disk is free on BOTH layers
                    if not _disk_is_free(grid, ix, iy, il, r_via):
                        continue
                    if not _disk_is_free(grid, ix, iy, jl, r_via):
                        continue

                    cand = (ix, iy, jl)
                    tg = gcur + via_cost
                    if tg + EPS < g_score.get(cand, float('inf')):
                        parent[cand] = cur
                        g_score[cand] = tg
                        push_state(cand, tg)


    raise RuntimeError("No route found")

def _path_cost(grid, rules, path):
    """A* cost used for comparison: unit/sqrt2 per grid move + via_cost per layer change."""
    if not path or len(path) < 2:
        return 0.0
    via_cost = float(rules.get("via_cost", 10.0))
    cost = 0.0
    for i in range(1, len(path)):
        (ix0, iy0, il0) = path[i-1]
        (ix1, iy1, il1) = path[i]
        if il1 != il0:
            cost += via_cost
        else:
            dx = abs(ix1 - ix0)
            dy = abs(iy1 - iy0)
            # 4/8-way grid: each step is 1 or sqrt(2)
            if dx == 1 and dy == 1:
                cost += math.sqrt(2)
            elif (dx == 1 and dy == 0) or (dx == 0 and dy == 1):
                cost += 1.0
            else:
                # fall back to Euclidean in rare compress/degenerate cases
                cost += math.hypot(dx, dy)
    return cost


def astar_route(grid, rules, start, goal):
    if start is None or goal is None:
        raise RuntimeError("Missing start or goal in astar_route")

    start_ix, start_iy = grid.world_to_grid(start["x"], start["y"])
    goal_ix,  goal_iy  = grid.world_to_grid(goal["x"],  goal["y"])

    if not grid.in_bounds(start_ix, start_iy) or not grid.in_bounds(goal_ix, goal_iy):
        raise RuntimeError("Start or goal out of grid bounds in astar_route")

    s_name = _normalize_layer_name(start.get("layer", grid.layers[0]), grid.layers, rules)
    g_name = _normalize_layer_name(goal.get("layer",  grid.layers[0]), grid.layers, rules)

    start_layer = grid.layers.index(s_name)
    goal_layer  = grid.layers.index(g_name)
    start_state = (start_ix, start_iy, start_layer)
    goal_state  = (goal_ix,  goal_iy,  goal_layer)

    best = None  # (cost, path, vias)

    # 1) Try planar (no vias)
    try:
        path0, vias0 = astar_search(
            grid, rules, start_state, goal_state,
            allow_via=False,
            start_xy=(start["x"], start["y"]),
            goal_xy=(goal["x"], goal["y"])
        )
        best = (_path_cost(grid, rules, path0), path0, vias0)
    except RuntimeError:
        pass

    # 2) Try with vias enabled
    try:
        path1, vias1 = astar_search(
            grid, rules, start_state, goal_state,
            allow_via=True,
            start_xy=(start["x"], start["y"]),
            goal_xy=(goal["x"], goal["y"])
        )
        cand = (_path_cost(grid, rules, path1), path1, vias1)
        if (best is None) or (cand[0] + 1e-9 < best[0]):
            best = cand
    except RuntimeError:
        pass

    if best is None:
        raise RuntimeError("No route found")

    _, path, vias = best
    return path, vias



# -----------------------
# (Old) DRC Engine (kept, but unused)
# -----------------------

def _local_to_world(lx, ly, px, py, rot_deg):
    t = math.radians(rot_deg or 0.0)
    ct, st = math.cos(t), math.sin(t)
    wx = px + lx*ct - ly*st
    wy = py + lx*st + ly*ct
    return wx, wy

def _dot(ax, ay, bx, by):
    return ax*bx + ay*by

def _clamp01(t):
    return 0.0 if t < 0.0 else (1.0 if t > 1.0 else t)

def _seg_seg_distance(p1, p2, q1, q2):
    x1,y1 = p1; x2,y2 = p2
    x3,y3 = q1; x4,y4 = q2
    ux, uy = x2-x1, y2-y1
    vx, vy = x4-x3, y4-y3
    wx, wy = x1-x3, y1-y3
    a = _dot(ux,uy,ux,uy)
    b = _dot(ux,uy,vx,vy)
    c = _dot(vx,vy,vx,vy)
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

    intersects = (dist < 1e-6) and (0.0 <= sc <= 1.0) and (0.0 <= tc <= 1.0)
    return dist, (cx1, cy1), (cx2, cy2), intersects

def _pt_seg_distance(pt, a, b):
    x,y = pt; x1,y1 = a; x2,y2 = b
    vx,vy = x2-x1,y2-y1
    if abs(vx) < 1e-12 and abs(vy) < 1e-12:
        return math.hypot(x-x1,y-y1), (x1,y1)
    t = _clamp01(((x-x1)*vx + (y-y1)*vy) / (vx*vx + vy*vy))
    cx,cy = x1 + t*vx, y1 + t*vy
    return math.hypot(x-cx, y-cy), (cx,cy)

def _dist_seg_to_circle(seg_a, seg_b, cx, cy, radius):
    d, cp = _pt_seg_distance((cx,cy), seg_a, seg_b)
    return d - radius, cp

def _rotate_to_local(x, y, px, py, rot_deg):
    t = math.radians(rot_deg or 0.0)
    ct, st = math.cos(t), math.sin(t)
    dx, dy = x - px, y - py
    lx =  dx*ct + dy*st
    ly = -dx*st + dy*ct
    return lx, ly

def _seg_rect_distance_local(a, b, hx, hy):
    for px,py in (a,b):
        if (-hx <= px <= hx) and (-hy <= py <= hy):
            d = -min(hx - abs(px), hy - abs(py))
            return d, (px,py)
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
    shape = pad.get("shape","rect")
    rot = float(pad.get("abs_rotation", pad.get("pad_rotation", 0.0)))
    px, py = pad["x"], pad["y"]

    if shape in ("rect","roundrect"):
        hx = float(pad.get("size_x",0.0))/2.0
        hy = float(pad.get("size_y",0.0))/2.0
        a_loc = _rotate_to_local(seg_a[0], seg_a[1], px, py, rot)
        b_loc = _rotate_to_local(seg_b[0], seg_b[1], px, py, rot)
        d, cp = _seg_rect_distance_local(a_loc, b_loc, hx, hy)
        cp_world = _local_to_world(cp[0], cp[1], px, py, rot)
        return d - (width/2.0), cp_world
    else:
        r = float(pad.get("radius", 0.2))
        d, cp = _dist_seg_to_circle(seg_a, seg_b, px, py, r)
        cp_world = _local_to_world(cp[0], cp[1], px, py, rot)
        return d - (width/2.0), cp_world

def _via_pad_clearance(cx, cy, via_diam, pad):
    shape = pad.get("shape","rect")
    rot = float(pad.get("abs_rotation", pad.get("pad_rotation", 0.0)))
    px, py = pad["x"], pad["y"]

    if shape in ("rect","roundrect"):
        hx = float(pad.get("size_x",0.0))/2.0
        hy = float(pad.get("size_y",0.0))/2.0
        v_loc = _rotate_to_local(cx, cy, px, py, rot)
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

    # small helper: open a circular window (radius in mm) as free space on a given layer index
    def _open_disk(ix, iy, il, radius_mm):
        r_cells = int(math.ceil(radius_mm / grid.step))
        for dx in range(-r_cells, r_cells + 1):
            for dy in range(-r_cells, r_cells + 1):
                if (dx * grid.step) ** 2 + (dy * grid.step) ** 2 <= radius_mm ** 2 + 1e-12:
                    jx, jy = ix + dx, iy + dy
                    if grid.in_bounds(jx, jy):
                        grid.clear_block(jx, jy, il)

    routes = []
    for task in tasks:
        net = task.get("net")
        net_id = task.get("net_id")
        start = task.get("start")
        goal  = task.get("goal")
        if not start or not goal:
            routes.append({"net": net, "net_id": net_id, "failed": True, "reason": "Missing start or goal"})
            continue

        rules_net, cls_name = rules_for_net(rules, net, net_id)

        # Candidate layers for start/goal (handle "*.Cu")
        def _cand_layers(layer_token):
            if layer_token == "*.Cu":
                return list(grid.layers)  # try all copper layers
            return [_normalize_layer_name(layer_token, grid.layers, rules_net)]

        start_layer_names = _cand_layers(start.get("layer", grid.layers[0]))
        goal_layer_names  = _cand_layers(goal.get("layer",  grid.layers[0]))

        # --- OPTIONAL: reuse an existing same-net via as an anchor/junction ---
        reuse_same_net_via = bool(rules_net.get("reuse_same_net_via", False))
        chosen_anchor = None
        anchor_layers = None
        if reuse_same_net_via and routes:
            candidates = []
            for rr in routes:
                if rr.get("failed") or rr.get("net_id") != net_id:
                    continue
                for v in rr.get("vias", []):
                    candidates.append(v)
            if candidates:
                sx, sy = start["x"], start["y"]
                def vd2(v):
                    vx, vy = float(v["at"][0]), float(v["at"][1])
                    return (vx - sx) ** 2 + (vy - sy) ** 2
                chosen_anchor = min(candidates, key=vd2)
                anchor_layers = (chosen_anchor.get("from"), chosen_anchor.get("to"))

        route_found = False
        last_error = None

        # Try all (startLayer, goalLayer) pairs until one succeeds
        for sL_name in start_layer_names:
            sL = grid.layers.index(sL_name)
            start_pad = find_pad_for_point(obstacles, start["x"], start["y"], sL_name)

            for gL_name in goal_layer_names:
                gL = grid.layers.index(gL_name)
                goal_pad = find_pad_for_point(obstacles, goal["x"], goal["y"], gL_name)

                # Open the pad copper (plus extra) on the specific attempt layers
                # If the pad is "*.Cu" (THT), open on ALL copper layers
                if start_pad:
                    if start_pad.get("layer") == "*.Cu":
                        for li in range(len(grid.layers)):
                            clear_full_pad_access(grid, start_pad, li, rules_net)
                    else:
                        clear_full_pad_access(grid, start_pad, sL, rules_net)

                if goal_pad:
                    if goal_pad.get("layer") == "*.Cu":
                        for li in range(len(grid.layers)):
                            clear_full_pad_access(grid, goal_pad, li, rules_net)
                    else:
                        clear_full_pad_access(grid, goal_pad,  gL, rules_net)

                # If we have an anchor via, open a small window at its location and steer goal to it (on sL)
                opened_anchor = False
                goal_try = {"x": goal["x"], "y": goal["y"], "layer": gL_name}
                if chosen_anchor:
                    vx, vy = float(chosen_anchor["at"][0]), float(chosen_anchor["at"][1])
                    via_size = float(chosen_anchor.get("size", rules_net.get("via_size", 0.6)))
                    clearance = float(rules_net.get("clearance", 0.2))
                    open_r = 0.5 * via_size + clearance
                    vix, viy = grid.world_to_grid(vx, vy)

                    for lname in anchor_layers:
                        if lname in grid.layers:
                            il = grid.layers.index(lname)
                            _open_disk(vix, viy, il, open_r)

                    goal_try = {"x": vx, "y": vy, "layer": sL_name}
                    opened_anchor = True

                try:
                    start_try = {"x": start["x"], "y": start["y"], "layer": sL_name}
                    path_cells, vias_raw = astar_route(grid, rules_net, start_try, goal_try)

                    # Restore pad blocking on success
                    def _restore_pad(pad, default_li):
                        if not pad: return
                        if pad.get("layer") == "*.Cu":
                            for li in range(len(grid.layers)):
                                set_pad_region(grid, pad, li)
                        else:
                            set_pad_region(grid, pad, default_li)

                    _restore_pad(start_pad, sL)
                    _restore_pad(goal_pad,  gL)

                    # Freeze this route (tracks + vias) as obstacles
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
                                "end":   [x1, y1],
                                "layer": l0,
                                "width": float(rules_net.get("trace_width", 0.25))
                            })

                    vias_out = []
                    for via in vias_raw:
                        vias_out.append({
                            "at":   [round(via["x"], 6), round(via["y"], 6)],
                            "from": via["from"],
                            "to":   via["to"],
                            "size": float(rules_net.get("via_size", 0.6)),
                            "drill":float(rules_net.get("via_drill", 0.3))
                        })

                    routes.append({
                        "net": net,
                        "net_id": net_id,
                        "netclass": cls_name,
                        "segments": segments,
                        "vias": vias_out
                    })

                    route_found = True
                    break  # gL loop
                except Exception as e:
                    last_error = str(e)
                    # Restore pad blocking before trying next pair
                    def _restore_pad_fail(pad, default_li):
                        if not pad: return
                        if pad.get("layer") == "*.Cu":
                            for li in range(len(grid.layers)):
                                set_pad_region(grid, pad, li)
                        else:
                            set_pad_region(grid, pad, default_li)

                    _restore_pad_fail(start_pad, sL)
                    _restore_pad_fail(goal_pad,  gL)
                    # continue to next (sL, gL)

            if route_found:
                break  # sL loop

        if not route_found:
            routes.append({
                "net": net,
                "net_id": net_id,
                "failed": True,
                "reason": last_error or "No route found",
                "netclass": cls_name
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
