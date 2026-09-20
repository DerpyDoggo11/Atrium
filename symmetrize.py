#!/usr/bin/env python3
"""
Atrium PCB — make the key layout (sensors + switches), the board/silk boundary,
and the sensor decoupling caps left/right symmetric about the board centre.

Left half is the MASTER: the right half is rebuilt as its exact mirror.
- de-dupes the stacked sensor pair (parks the extra),
- parks the orphan switch that has no sensor,
- moves the sensor decoupling caps so one sits next to each sensor,
- rewrites the Edge.Cuts outline + F.SilkS bottom-section curve symmetric,
- prints the new SILK_POLY / BOARD_OUTLINE to paste into led_grid.py.

Geometry (board outline, silk curve, centre axis) is read live from the PCB.

    python symmetrize.py            # dry run + stats
    python symmetrize.py --preview  # + preview PNG
    python symmetrize.py --apply    # write (timestamped backup first)
"""
import os, re, math, argparse, datetime, uuid

HERE = os.path.dirname(os.path.abspath(__file__))
PCB_PATH = os.path.join(HERE, "Atrium PCB", "Atrium PCB.kicad_pcb")

MASTER = "left"                # half to mirror from
SENSOR_VALUE = "CH604ASR"
SWITCH_VALUE = "SW_Push"
CAP_VALUE = "100nF"
SENSOR_NETS = frozenset({"+3.3V", "GND"})
DEDUP_MM = 2.0
COLOCATE_MM = 1.5
SENSOR_CAP_OFFSET = (0.0, -3.2)   # cap (dx,dy) relative to its sensor
CORE_IC_VALUES = {"RP2350A", "CD74HC4067SM", "W25Q128JVS", "AP2112K-3.3",
                  "SN74LVC1T45DBVR", "LSM6DS3TR-C", "USB-TYPE-C-018"}
CORE_KEEP_MM = 13.0
PARK_X, PARK_Y, PARK_STEP = 6.0, 135.0, 7.0

CENTER_X = None   # computed from the board outline at run time


# ----------------------------- parsing -----------------------------
def footprint_spans(txt):
    out = []; i = 0
    while True:
        j = txt.find('\n\t(footprint ', i)
        if j < 0: break
        k = j + 1; depth = 0; started = False
        while k < len(txt):
            c = txt[k]
            if c == '(': depth += 1; started = True
            elif c == ')':
                depth -= 1
                if depth == 0 and started:
                    out.append((j + 1, k + 1)); break
            k += 1
        i = k + 1
    return out

def parse(txt):
    fps = {}
    for s, e in footprint_spans(txt):
        b = txt[s:e]
        ref = re.search(r'\(property "Reference" "([^"]+)"', b)
        if not ref: continue
        ref = ref.group(1)
        val = re.search(r'\(property "Value" "([^"]*)"', b)
        at = re.search(r'\n\t\t\(at ([-0-9.]+) ([-0-9.]+)(?: ([-0-9.]+))?\)', b)
        nets = frozenset(re.findall(r'\(net "([^"]+)"\)', b))
        fps[ref] = {"span": (s, e), "value": val.group(1) if val else "",
                    "x": float(at.group(1)), "y": float(at.group(2)),
                    "rot": float(at.group(3)) if at.group(3) else 0.0, "nets": nets}
    return fps

def board_gr_lines(txt):
    """(layer, (ax,ay), (bx,by), full_block_text) for every board-level gr_line."""
    res = []
    for m in re.finditer(
        r'\t\(gr_line\n\t\t\(start ([-0-9.]+) ([-0-9.]+)\)\n\t\t\(end ([-0-9.]+) ([-0-9.]+)\)\n'
        r'.*?\(layer "([^"]+)"\)\n\t\t\(uuid "[^"]*"\)\n\t\)\n', txt, re.S):
        res.append((m.group(5), (float(m.group(1)), float(m.group(2))),
                    (float(m.group(3)), float(m.group(4))), m.group(0)))
    return res

def order_polyline(segs):
    """Chain undirected segments into one ordered vertex list."""
    from collections import defaultdict
    adj = defaultdict(list)
    key = lambda p: (round(p[0], 2), round(p[1], 2))   # 0.01mm snap: segment ends don't match exactly
    for a, b in segs:
        adj[key(a)].append(b); adj[key(b)].append(a)
    ends = [p for p, nb in adj.items() if len(nb) == 1]
    start = ends[0] if ends else key(segs[0][0])
    poly = [start]; prev = None; cur = start
    seen = set()
    while True:
        nxts = [n for n in adj[cur] if key(n) != prev and key(n) not in seen]
        if not nxts: break
        nxt = nxts[0]; poly.append(nxt); seen.add(cur); prev = cur; cur = key(nxt)
        if cur == start: break
    return poly


# ----------------------------- geometry -----------------------------
def mirror(p): return (round(2 * CENTER_X - p[0], 4), p[1])

def symmetrize_open(poly):
    """Symmetric open polyline built from the left half of `poly`."""
    left = [p for p in poly if p[0] <= CENTER_X - 1e-6]
    left = sorted(left, key=lambda p: poly.index(p))   # keep original order
    right = [mirror(p) for p in reversed(left)]
    return [tuple(p) for p in left] + right

def symmetric_board(board_poly):
    """Symmetric closed outline from the left half (excludes the on-axis bottom vertex)."""
    left = [p for p in board_poly if p[0] < CENTER_X - 1.0]   # 1mm: drop near-axis bottom vertex
    tl = min(left, key=lambda p: (p[1], p[0]))                # top-left = smallest y
    others = [p for p in left if p != tl]
    lv = max(others, key=lambda p: p[1]) if others else tl    # left vertex = largest y
    bottom_y = max(p[1] for p in board_poly)
    tr = mirror(tl); rv = mirror(lv)
    return [tuple(tl), tuple(tr), tuple(rv), (round(CENTER_X, 4), bottom_y), tuple(lv)]

def fill_polygon(board_out, silk_poly):
    """Region LEDs must stay inside: board top+sides down to the silk curve."""
    tl, tr = board_out[0], board_out[1]
    # silk_poly runs left->right along the bottom boundary; walk it right->left
    silk_rl = list(reversed(silk_poly))
    return [tl, tr] + silk_rl


def set_pos(block, x, y, rot):
    rep = (f'\n\t\t(at {x} {y} {int(rot) if float(rot).is_integer() else rot})'
           if rot else f'\n\t\t(at {x} {y})')
    return re.sub(r'\n\t\t\(at [-0-9.]+ [-0-9.]+(?: [-0-9.]+)?\)', rep, block, count=1)


def rewrite_boundaries(txt, board_out, silk_poly, edge_blocks, silk_blocks):
    """Remove the exact current Edge.Cuts/F.SilkS boundary line blocks, add symmetric ones."""
    for blk in edge_blocks + silk_blocks:
        assert blk in txt, "boundary block not found (already modified?)"
        txt = txt.replace(blk, "", 1)

    def emit(poly, layer, closed, width):
        seq = list(poly) + ([poly[0]] if closed else [])
        s = ""
        for i in range(len(seq) - 1):
            a, b = seq[i], seq[i + 1]
            s += (f'\t(gr_line\n\t\t(start {a[0]:g} {a[1]:g})\n\t\t(end {b[0]:g} {b[1]:g})\n'
                  f'\t\t(stroke\n\t\t\t(width {width})\n\t\t\t(type solid)\n\t\t)\n'
                  f'\t\t(layer "{layer}")\n\t\t(uuid "{uuid.uuid4()}")\n\t)\n')
        return s
    add = emit(board_out, "Edge.Cuts", True, 0.1) + emit(silk_poly, "F.SilkS", False, 0.15)
    k = txt.rstrip().rfind(")")
    return txt[:k] + add + txt[k:]


# ----------------------------- main -----------------------------
def main():
    global CENTER_X
    ap = argparse.ArgumentParser()
    ap.add_argument("--apply", action="store_true")
    ap.add_argument("--preview", action="store_true")
    args = ap.parse_args()

    txt = open(PCB_PATH, encoding="utf-8").read()
    fps = parse(txt)

    gl = board_gr_lines(txt)
    edge_segs = [(a, b) for lyr, a, b, blk in gl if lyr == "Edge.Cuts"]
    silk_segs = [(a, b) for lyr, a, b, blk in gl if lyr == "F.SilkS"]
    edge_blocks = [blk for lyr, a, b, blk in gl if lyr == "Edge.Cuts"]
    silk_blocks = [blk for lyr, a, b, blk in gl if lyr == "F.SilkS"]
    board_poly = order_polyline(edge_segs)
    silk_line = order_polyline(silk_segs)
    top_y0 = [p for p in board_poly if abs(p[1]) < 1e-6]
    CENTER_X = round(sum(p[0] for p in top_y0) / len(top_y0), 6)
    print(f"CENTER_X (from top edge) = {CENTER_X}")

    sym_board = symmetric_board(board_poly)
    sym_silk = symmetrize_open(silk_line)
    sym_fill = fill_polygon(sym_board, sym_silk)

    # sensors / switches / keys
    sensors = {r: f for r, f in fps.items() if f["value"] == SENSOR_VALUE}
    switches = {r: f for r, f in fps.items() if f["value"] == SWITCH_VALUE}

    dup_park = set(); srefs = list(sensors)
    for i in range(len(srefs)):
        for j in range(i + 1, len(srefs)):
            a, b = sensors[srefs[i]], sensors[srefs[j]]
            if math.hypot(a["x"] - b["x"], a["y"] - b["y"]) < DEDUP_MM:
                _, drop = sorted([srefs[i], srefs[j]], key=lambda r: int(re.sub(r"\D", "", r)))
                dup_park.add(drop)
    orphan_sw = {r for r, w in switches.items()
                 if min(math.hypot(w["x"] - s["x"], w["y"] - s["y"]) for s in sensors.values()) > COLOCATE_MM}

    active_sensors = {r: s for r, s in sensors.items() if r not in dup_park}
    active_switches = {r: w for r, w in switches.items() if r not in orphan_sw}

    keys = []; used_sw = set()
    for sr, s in active_sensors.items():
        cand = sorted(((math.hypot(w["x"] - s["x"], w["y"] - s["y"]), r)
                       for r, w in active_switches.items() if r not in used_sw))
        if not cand or cand[0][0] > COLOCATE_MM: continue
        wr = cand[0][1]; used_sw.add(wr); w = switches[wr]
        keys.append({"s": sr, "w": wr, "sx": s["x"], "sy": s["y"], "srot": s["rot"],
                     "wx": w["x"], "wy": w["y"], "wrot": w["rot"]})

    left = [k for k in keys if k["sx"] < CENTER_X]
    right = [k for k in keys if k["sx"] >= CENTER_X]
    master, slave = (left, right) if MASTER == "left" else (right, left)

    slots = [{"sx": round(2 * CENTER_X - k["sx"], 4), "sy": k["sy"], "srot": (-k["srot"]) % 360,
              "wx": round(2 * CENTER_X - k["wx"], 4), "wy": k["wy"], "wrot": (-k["wrot"]) % 360}
             for k in master]

    free = list(range(len(slots))); assign = {}
    for k in sorted(slave, key=lambda k: (k["sy"], k["sx"])):
        if not free: break
        bi = min(free, key=lambda i: math.hypot(slots[i]["sx"] - k["sx"], slots[i]["sy"] - k["sy"]))
        assign[k["s"]] = bi; free.remove(bi)

    moves = {}
    for k in slave:
        if k["s"] not in assign: continue
        sl = slots[assign[k["s"]]]
        moves[k["s"]] = (sl["sx"], sl["sy"], sl["srot"])
        moves[k["w"]] = (sl["wx"], sl["wy"], sl["wrot"])

    park = list(dup_park) + list(orphan_sw) + [r for k in slave if k["s"] not in assign for r in (k["s"], k["w"])]
    for i, pr in enumerate(park):
        moves[pr] = (PARK_X + i * PARK_STEP, PARK_Y, 0.0)

    # final sensor positions (master fixed, slave moved)
    final_sensors = {k["s"]: (k["sx"], k["sy"]) for k in master}
    for k in slave:
        if k["s"] in assign:
            sl = slots[assign[k["s"]]]; final_sensors[k["s"]] = (sl["sx"], sl["sy"])

    # sensor caps: keep core, move rest one-per-sensor
    core = [(f["x"], f["y"]) for f in fps.values() if f["value"] in CORE_IC_VALUES]
    scaps = sorted((r for r, f in fps.items()
                    if f["value"] == CAP_VALUE and f["nets"] == SENSOR_NETS
                    and not any(math.hypot(f["x"] - ix, f["y"] - iy) < CORE_KEEP_MM for ix, iy in core)),
                   key=lambda r: int(re.sub(r"\D", "", r) or 0))
    for i, (sr, (sx, sy)) in enumerate(final_sensors.items()):
        if i < len(scaps):
            moves[scaps[i]] = (round(sx + SENSOR_CAP_OFFSET[0], 4), round(sy + SENSOR_CAP_OFFSET[1], 4), 0.0)

    print(f"keys: {len(keys)} (L{len(left)}/R{len(right)}) -> symmetric {len(master)*2}")
    print(f"parked: {sorted(dup_park)} + {sorted(orphan_sw)}")
    print(f"sensor caps relocated: {min(len(scaps), len(final_sensors))}")
    print(f"footprint moves: {len(moves)}")
    print("BOARD_OUTLINE =", sym_board)
    print("SILK_POLY =", [tuple(round(v, 4) for v in p) for p in sym_fill])

    if args.preview:
        preview(fps, moves, sym_board, sym_silk, sym_fill)
    if not args.apply:
        print("\n(dry run — --apply to write)")
        return

    stamp = datetime.datetime.now().strftime("%Y%m%d-%H%M%S")
    open(f"{PCB_PATH}.{stamp}.bak", "w", encoding="utf-8").write(txt)
    edits = [(fps[r]["span"][0], fps[r]["span"][1], set_pos(txt[fps[r]["span"][0]:fps[r]["span"][1]], *mv))
             for r, mv in moves.items()]
    edits.sort(reverse=True)
    new = txt
    for s, e, b in edits:
        new = new[:s] + b + new[e:]
    new = rewrite_boundaries(new, sym_board, sym_silk, edge_blocks, silk_blocks)
    assert new.count("(") == new.count(")"), "paren imbalance"
    open(PCB_PATH, "w", encoding="utf-8").write(new)
    print(f"\napplied (backup .{stamp}.bak). Paste the BOARD_OUTLINE/SILK_POLY above into led_grid.py.")


def preview(fps, moves, board, silk, fill):
    try:
        import matplotlib; matplotlib.use("Agg"); import matplotlib.pyplot as plt
    except ImportError:
        print("(no matplotlib)"); return
    fig, ax = plt.subplots(figsize=(17, 8))
    b = list(board) + [board[0]]
    ax.plot([p[0] for p in b], [p[1] for p in b], 'k-', lw=2)
    ax.plot([p[0] for p in silk], [p[1] for p in silk], 'm--', lw=1.2)
    ax.plot([p[0] for p in fill] + [fill[0][0]], [p[1] for p in fill] + [fill[0][1]], 'g:', lw=0.8)
    ax.axvline(CENTER_X, color='0.7', ls=':')
    for r, f in fps.items():
        x, y, _ = moves.get(r, (f["x"], f["y"], 0))
        if y >= 120: continue
        if f["value"] == SENSOR_VALUE: ax.scatter([x], [y], c='red', s=45, marker='x', zorder=5)
        elif f["value"] == SWITCH_VALUE: ax.scatter([x], [y], facecolors='none', edgecolors='0.5', s=200, zorder=3)
        elif r in moves and f["value"] == CAP_VALUE: ax.scatter([x], [y], c='deepskyblue', s=14, zorder=4)
    ax.set_aspect('equal'); ax.invert_yaxis()
    ax.set_title(f"Symmetric (axis x={CENTER_X}): sensors(x) switches(o) sensor-caps(.)")
    out = os.path.join(HERE, "symmetrize_preview.png")
    plt.savefig(out, dpi=90, bbox_inches='tight'); print("preview:", out)


if __name__ == "__main__":
    main()
