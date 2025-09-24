import json
import uuid
import sys
import os

def _pick_seg_points(seg):
    """
    Support both formats:
      {"start":[x,y], "end":[x,y], "layer":"F.Cu", "width":0.15}
      {"from":[x,y],  "to":[x,y],  "layer":"F.Cu", "width":0.15}
    """
    if "start" in seg and "end" in seg:
        return seg["start"], seg["end"]
    if "from" in seg and "to" in seg:
        return seg["from"], seg["to"]
    raise KeyError("segment missing both ('start','end') and ('from','to')")

def _unique_out_path(path):
    """
    If 'path' exists, generate 'name(n).ext' that doesn't exist.
    e.g. sample_routed.kicad_pcb -> sample_routed(1).kicad_pcb, etc.
    """
    base, ext = os.path.splitext(path)
    if not os.path.exists(path):
        return path
    n = 1
    while True:
        cand = f"{base}({n}){ext}"
        if not os.path.exists(cand):
            return cand
        n += 1

def _assert_not_same_file(in_path, out_path):
    # Prevent accidental overwrite if user passes same file twice
    in_abs = os.path.abspath(in_path)
    out_abs = os.path.abspath(out_path)
    if in_abs == out_abs:
        raise RuntimeError("Output file must be different from input .kicad_pcb")

def json_to_kicad(json_file, pcb_file, out_file):
    with open(json_file, "r", encoding="utf-8") as f:
        data = json.load(f)

    # If router returned DRC errors instead of routes, abort with a helpful message.
    if "routes" not in data:
        raise RuntimeError(
            "No 'routes' found in JSON. "
            "It looks like the router returned DRC errors or a summary instead."
        )

    # Build KiCad S-expressions for segments and vias
    kicad_lines = []
    for route in data.get("routes", []):
        net_id = route.get("net_id")
        if net_id is None:
            # KiCad needs a numeric net id
            raise RuntimeError(f"Route for net '{route.get('net')}' is missing 'net_id'.")

        # Segments
        for seg in route.get("segments", []):
            try:
                (start_x, start_y), (end_x, end_y) = _pick_seg_points(seg)
            except KeyError as e:
                raise RuntimeError(f"Bad segment format for net_id {net_id}: {e}")
            width = seg.get("width", 0.25)
            layer = seg["layer"]
            line = f'''  (segment
    (start {start_x} {start_y})
    (end {end_x} {end_y})
    (width {width})
    (layer "{layer}")
    (net {net_id})
    (uuid "{uuid.uuid4()}")
  )'''
            kicad_lines.append(line)

        # Vias
        for via in route.get("vias", []):
            at_x, at_y = via["at"]
            vsize = via.get("size", 0.8)
            vdrill = via.get("drill", 0.4)
            vfrom = via.get("from", "F.Cu")
            vto   = via.get("to",   "B.Cu")
            # KiCad allows (layers "F.Cu" "In1.Cu") etc. for micro/blind/buried; pair is fine.
            line = f'''  (via
    (at {at_x} {at_y})
    (size {vsize})
    (drill {vdrill})
    (layers "{vfrom}" "{vto}")
    (net {net_id})
    (uuid "{uuid.uuid4()}")
  )'''
            kicad_lines.append(line)

    # Read the existing PCB file
    with open(pcb_file, "r", encoding="utf-8") as f:
        pcb_data = f.read().rstrip()

    # Find a good insertion point: just before the last ')'
    # Fallback: if not found, append at end and add a closing parenthesis.
    insert_ok = pcb_data.endswith(")")
    if insert_ok:
        pcb_body = pcb_data[:-1].rstrip()
        new_pcb  = pcb_body + "\n" + "\n".join(kicad_lines) + "\n)"
    else:
        new_pcb  = pcb_data + "\n" + "\n".join(kicad_lines) + "\n)"

    # Ensure we don't overwrite existing files (and not same as input file)
    _assert_not_same_file(pcb_file, out_file)
    safe_out = _unique_out_path(out_file)

    with open(safe_out, "w", encoding="utf-8") as f:
        f.write(new_pcb)

    print(f"✅ New routing injected into {safe_out}")

if __name__ == "__main__":
    if len(sys.argv) < 4:
        print("Usage: python inject_routes.py <output.json> <input.kicad_pcb> <output.kicad_pcb>")
        sys.exit(1)

    json_file = sys.argv[1]
    pcb_file  = sys.argv[2]
    out_file  = sys.argv[3]
    json_to_kicad(json_file, pcb_file, out_file)
