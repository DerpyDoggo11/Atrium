#!/usr/bin/env python3
"""
Atrium PCB — LED backlight grid placer.

Places the addressable SK6805 LEDs (and their 100nF decoupling caps) as a regular
grid behind the keys, going AROUND the hall-effect sensors but IGNORING the
key-switch / keycap holes and areas. Rows are anchored to the sensor Y-levels.

Everything is driven by the PARAMETERS block below — edit and re-run.

Usage:
    python led_grid.py            # dry run: prints stats (+ --preview for a PNG)
    python led_grid.py --preview  # dry run + write preview PNG
    python led_grid.py --apply    # write the new positions into the .kicad_pcb
                                   # (a timestamped backup is made first)

The script only ever MOVES existing footprints and toggles their reference-label
visibility. It never edits the schematic, nets, or component count.
"""
import os, re, math, argparse, datetime, sys

# ============================== PARAMETERS ==============================
HERE = os.path.dirname(os.path.abspath(__file__))
PCB_PATH = os.path.join(HERE, "Atrium PCB", "Atrium PCB.kicad_pcb")

PITCH_X   = 6.0     # mm  horizontal LED spacing (columns)
PITCH_Y   = 6.0     # mm  vertical row spacing (6.0 → a row sits above the top sensors)

# Rows/columns are aligned to a reference sensor so one row sits exactly at the
# top sensor's Y level (row above = -PITCH_Y, row below = +PITCH_Y, ...).
ROW_ANCHOR_MODE = "top_sensor"   # "top_sensor" -> use min-Y sensor; or a number (mm)
COL_ANCHOR_MODE = "top_sensor"   # "top_sensor" -> column phase through that sensor's X; or a number (mm)

# Go AROUND the hall sensors: skip any grid cell whose LED (or its cap) would sit
# within these center-to-center distances of a sensor body. Switch/keycap holes
# are IGNORED entirely (not treated as keepouts).
SENSOR_KEEPOUT_LED = 3.4  # mm  LED center -> nearest sensor center (0 would put LEDs on the sensors)
SENSOR_KEEPOUT_CAP = 3.6  # mm  LED-cap center -> nearest sensor center (caps clip SOT-23 pads if too small)
# The sensor decoupling caps now sit ON the sensors (symmetrize.py). Keep the grid off them too.
SCAP_KEEPOUT_LED   = 2.6  # mm  LED center -> nearest sensor-cap center
SCAP_KEEPOUT_CAP   = 2.7  # mm  LED-cap center -> nearest sensor-cap center

EDGE_MARGIN   = 1.0   # mm  keep LED centers this far inside the silkscreen boundary
BOARD_MARGIN  = 1.5   # mm  keep LED & cap centers this far inside the board edge (copper)

LED_ROT    = 180      # deg LED rotation (flipped 180 so DIN/DOUT face the shifted cap)

# Decoupling cap placement relative to its LED (kept as a tight LED+cap group).
CAP_SIDE   = "above"  # "below" or "above" the LED
CAP_OFFSET = 2.1      # mm  distance from LED center to cap center
CAP_ROT    = 0        # deg cap rotation (0 when above the flipped LED)

HIDE_ALL_LABELS = True   # hide every footprint's reference designator

# Where to park LEDs/caps that don't fit on the board (kept, just moved aside).
PARK_X, PARK_Y, PARK_STEP = 6.0, 120.0, 7.0

# Silkscreen fill boundary (region the grid must stay inside = board minus the
# bottom-section silk curve) and the board outline (Edge.Cuts). Edit if the board
# outline changes.
# (regenerated 2026-09-17 from the reshaped board; symmetric axis x=129.4819,
#  chamfered top corners, trapezoidal bottom with flat notch)
SILK_POLY = [(3.0,0.0),(255.965,0.0),(258.965,3.0),(258.964,54.82),(254.057,54.8224),
             (188.229,65.15),(192.04,85.15),(139.629,95.15),(119.335,95.15),
             (66.924,85.15),(70.735,65.15),(4.907,54.8224),(0.0,54.82),(0.001,3.0)]
BOARD_OUTLINE = [(3.0,0.0),(255.965,0.0),(258.965,3.0),(258.993,77.0),(257.846,78.623),
                 (132.008,100.854),(127.0,100.862),(1.148,78.628),(0.0,77.0),(0.001,3.0)]

SPK_IGNORE = {"SPK1"}         # components whose keepout is ignored (grid may sit over them)
LED_VALUE = "SK6805-EC15"     # addressable LED value
SENSOR_VALUE = "CH604ASR"     # hall sensor value
CAP_VALUE = "100nF"           # decoupling cap value
LED_POWER_NET, GND_NET = "+5V_LED", "GND"   # nets identifying an LED decoupling cap
# =======================================================================


# ----------------------------- geometry helpers -----------------------------
def point_in_poly(x, y, poly):
    inside = False; n = len(poly); j = n - 1
    for i in range(n):
        xi, yi = poly[i]; xj, yj = poly[j]
        if ((yi > y) != (yj > y)) and (x < (xj - xi) * (y - yi) / (yj - yi) + xi):
            inside = not inside
        j = i
    return inside

def dist_to_poly(x, y, poly):
    dm = 1e9; n = len(poly)
    for i in range(n):
        ax, ay = poly[i]; bx, by = poly[(i + 1) % n]
        dx, dy = bx - ax, by - ay
        t = ((x - ax) * dx + (y - ay) * dy) / (dx * dx + dy * dy) if (dx or dy) else 0
        t = max(0, min(1, t)); px, py = ax + t * dx, ay + t * dy
        dm = min(dm, math.hypot(x - px, y - py))
    return dm


# ----------------------------- PCB parsing -----------------------------
def footprint_spans(txt):
    """Yield (start, end) char offsets of every top-level (footprint ...) block."""
    out = []; i = 0
    while True:
        j = txt.find('\n\t(footprint ', i)
        if j < 0:
            break
        k = j + 1; depth = 0; started = False
        while k < len(txt):
            c = txt[k]
            if c == '(':
                depth += 1; started = True
            elif c == ')':
                depth -= 1
                if depth == 0 and started:
                    out.append((j + 1, k + 1)); break
            k += 1
        i = k + 1
    return out

def ref_of(block):
    m = re.search(r'\(property "Reference" "([^"]+)"', block); return m.group(1) if m else None

def val_of(block):
    m = re.search(r'\(property "Value" "([^"]*)"', block); return m.group(1) if m else ""

def pad_nets(block):
    """{pad_number: net_name} for the numbered pads of a footprint."""
    nets = {}
    for pm in re.finditer(r'\(pad "(\w+)"[^\n]*(?:\n(?!\s*\(pad ).*)*?\(net "([^"]+)"\)', block):
        nets.setdefault(pm.group(1), pm.group(2))
    return nets

def parse_pcb(txt):
    """Return dicts of footprints keyed by reference with position + nets."""
    fps = {}
    for s, e in footprint_spans(txt):
        b = txt[s:e]
        ref = ref_of(b)
        if not ref:
            continue
        at = re.search(r'\n\t\t\(at ([-0-9.]+) ([-0-9.]+)(?: ([-0-9.]+))?\)', b)
        fps[ref] = {
            "span": (s, e), "value": val_of(b),
            "x": float(at.group(1)), "y": float(at.group(2)),
            "rot": float(at.group(3)) if at.group(3) else 0.0,
            "nets": pad_nets(b),
        }
    return fps


# ----------------------------- chain + cap discovery -----------------------------
def led_chain_order(fps):
    """LEDs in electrical DIN->DOUT order (orphans appended)."""
    leds = [r for r, f in fps.items() if f["value"] == LED_VALUE]
    din  = {r: fps[r]["nets"].get("1") for r in leds}
    dout = {r: fps[r]["nets"].get("3") for r in leds}
    by_din = {}
    for r in leds:
        by_din.setdefault(din[r], []).append(r)
    dout_nets = set(dout.values())
    heads = [r for r in leds if din[r] not in dout_nets]

    def walk(start, seen):
        seq = [start]; seen.add(start); cur = start
        while True:
            nxt = [x for x in by_din.get(dout[cur], []) if x not in seen]
            if not nxt:
                break
            cur = nxt[0]; seq.append(cur); seen.add(cur)
        return seq

    seen = set(); chains = []
    for h in sorted(heads, key=lambda r: (r not in by_din, r)):
        if h not in seen:
            chains.append(walk(h, seen))
    chains.sort(key=len, reverse=True)
    order = [r for ch in chains for r in ch]
    for r in leds:                      # any left out (fully isolated)
        if r not in order:
            order.append(r)
    return order

def led_caps(fps):
    """Decoupling caps (value + exactly the two LED power nets), sorted by ref number."""
    out = []
    for r, f in fps.items():
        if f["value"] != CAP_VALUE:
            continue
        if set(f["nets"].values()) == {LED_POWER_NET, GND_NET}:
            out.append(r)
    return sorted(out, key=lambda r: int(re.sub(r"\D", "", r) or 0))


# ----------------------------- grid build -----------------------------
def component_obstacles(txt, fps, sensors):
    """Keepout circles (x,y,radius) for non-grid, non-switch components inside the fill region.
    Radius = half-diagonal of the footprint's pad/body bounding box + LED reach + clearance.
    (Sensor decoupling caps sitting on a sensor are excluded; other +3.3V caps — e.g. the amp's —
    are kept as obstacles.)"""
    import math as _m
    near_sensor = lambda f: any(_m.hypot(f["x"] - sx, f["y"] - sy) < 6 for sx, sy in sensors)
    obs = []
    for s, e in footprint_spans(txt):
        b = txt[s:e]
        ref = re.search(r'\(property "Reference" "([^"]+)"', b)
        if not ref: continue
        ref = ref.group(1); f = fps.get(ref)
        if not f: continue
        if ref in SPK_IGNORE: continue   # ignore buzzer/speaker boundary — LEDs may sit over it
        if f["value"] in (LED_VALUE, "SW_Push", SENSOR_VALUE): continue
        if ref.startswith("C") and set(f["nets"].values()) == {LED_POWER_NET, GND_NET}: continue
        if f["value"] == CAP_VALUE and set(f["nets"].values()) == {"+3.3V", GND_NET} and near_sensor(f): continue
        if not point_in_poly(f["x"], f["y"], SILK_POLY): continue
        reach = 0.0
        for pm in re.finditer(r'\(pad "[^"]*" \S+ \S+\s*\(at ([-0-9.]+) ([-0-9.]+)(?: [-0-9.]+)?\)\s*\(size ([-0-9.]+) ([-0-9.]+)\)', b):
            px, py, w, h = map(float, pm.groups())
            reach = max(reach, _m.hypot(abs(px) + w / 2, abs(py) + h / 2))
        # also the body/courtyard extent (a big TH speaker's body dwarfs its pads)
        for cm in re.finditer(r'\(fp_circle\n\t\t\(center ([-0-9.]+) ([-0-9.]+)\)\n\t\t\(end ([-0-9.]+) ([-0-9.]+)\)[\s\S]*?\(layer "F\.(?:CrtYd|Fab)"', b):
            cx, cy, ex, ey = map(float, cm.groups())
            reach = max(reach, _m.hypot(cx, cy) + _m.hypot(ex - cx, ey - cy))
        for lm in re.finditer(r'\(fp_(?:line|rect)\n\t\t\(start ([-0-9.]+) ([-0-9.]+)\)\n\t\t\(end ([-0-9.]+) ([-0-9.]+)\)[\s\S]*?\(layer "F\.CrtYd"', b):
            for px, py in ((float(lm.group(1)), float(lm.group(2))), (float(lm.group(3)), float(lm.group(4)))):
                reach = max(reach, _m.hypot(px, py))
        obs.append((f["x"], f["y"], reach + 1.1))   # + LED half-diagonal + clearance
    return obs


def build_grid(sensors, sensor_caps, obstacles=()):
    """Serpentine list of (x, y) LED cells, anchored to the sensors, avoiding sensors + sensor caps + components."""
    top_sensor = min(sensors, key=lambda s: (s[1], s[0]))
    y_anchor = top_sensor[1] if ROW_ANCHOR_MODE == "top_sensor" else float(ROW_ANCHOR_MODE)
    x_anchor = top_sensor[0] if COL_ANCHOR_MODE == "top_sensor" else float(COL_ANCHOR_MODE)

    ys = [p[1] for p in SILK_POLY]; xs = [p[0] for p in SILK_POLY]
    ymin, ymax, xmin, xmax = min(ys) - 2, max(ys) + 2, min(xs) - 4, max(xs) + 4

    cap_dir = 1.0 if CAP_SIDE == "below" else -1.0   # +y is downward in KiCad

    def clear(x, y, sr, cr):
        return (all(math.hypot(x - sx, y - sy) >= sr for sx, sy in sensors) and
                all(math.hypot(x - sx, y - sy) >= cr for sx, sy in sensor_caps))

    def obst_clear(x, y):
        return all(math.hypot(x - ox, y - oy) >= r for ox, oy, r in obstacles)

    def cell_ok(x, y):
        if not point_in_poly(x, y, SILK_POLY): return False
        if dist_to_poly(x, y, SILK_POLY) < EDGE_MARGIN: return False
        if dist_to_poly(x, y, BOARD_OUTLINE) < BOARD_MARGIN: return False
        if not clear(x, y, SENSOR_KEEPOUT_LED, SCAP_KEEPOUT_LED): return False
        if not obst_clear(x, y): return False
        cx, cy = x, y + cap_dir * CAP_OFFSET          # paired cap position
        if not point_in_poly(cx, cy, SILK_POLY): return False
        if dist_to_poly(cx, cy, BOARD_OUTLINE) < BOARD_MARGIN: return False
        if not clear(cx, cy, SENSOR_KEEPOUT_CAP, SCAP_KEEPOUT_CAP): return False
        if not obst_clear(cx, cy): return False
        return True

    # row indices spanning the polygon, anchored so n=0 -> y_anchor
    n_lo = math.floor((ymin - y_anchor) / PITCH_Y)
    n_hi = math.ceil((ymax - y_anchor) / PITCH_Y)
    cells = []
    for ridx, n in enumerate(range(n_lo, n_hi + 1)):
        y = y_anchor + n * PITCH_Y
        if not (ymin <= y <= ymax):
            continue
        m_lo = math.floor((xmin - x_anchor) / PITCH_X)
        m_hi = math.ceil((xmax - x_anchor) / PITCH_X)
        row_xs = [x_anchor + m * PITCH_X for m in range(m_lo, m_hi + 1)]
        if ridx % 2:                                   # serpentine
            row_xs = row_xs[::-1]
        for x in row_xs:
            if cell_ok(x, y):
                cells.append((round(x, 4), round(y, 4)))
    return cells, (x_anchor, y_anchor)


# ----------------------------- block edits -----------------------------
def set_position(block, x, y, rot=0.0):
    if rot:
        rep = f'\n\t\t(at {x} {y} {int(rot) if float(rot).is_integer() else rot})'
    else:
        rep = f'\n\t\t(at {x} {y})'
    return re.sub(r'\n\t\t\(at [-0-9.]+ [-0-9.]+(?: [-0-9.]+)?\)', rep, block, count=1)

def hide_reference(block):
    """Ensure the Reference property carries (hide yes) (idempotent)."""
    m = re.search(r'(\(property "Reference" "[^"]*"\n(\t+)\(at [^\n]*\)\n)', block)
    if not m:
        return block
    after = block[m.end():m.end() + 40]
    if after.lstrip().startswith("(hide"):
        return block
    return block[:m.end()] + m.group(2) + "(hide yes)\n" + block[m.end():]


# ----------------------------- main -----------------------------
def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--apply", action="store_true", help="write changes to the .kicad_pcb")
    ap.add_argument("--preview", action="store_true", help="render a preview PNG")
    args = ap.parse_args()

    txt = open(PCB_PATH, encoding="utf-8").read()
    fps = parse_pcb(txt)
    sensors = [(f["x"], f["y"]) for f in fps.values() if f["value"] == SENSOR_VALUE]
    # sensor decoupling caps now sit on the sensors (+3.3V/GND, within a few mm of a sensor)
    sensor_caps = [(f["x"], f["y"]) for r, f in fps.items()
                   if f["value"] == CAP_VALUE and set(f["nets"].values()) == {"+3.3V", "GND"}
                   and any(math.hypot(f["x"] - sx, f["y"] - sy) < 6 for sx, sy in sensors)]
    chain = led_chain_order(fps)
    caps = led_caps(fps)
    obstacles = component_obstacles(txt, fps, sensors)
    cells, anchor = build_grid(sensors, sensor_caps, obstacles)

    n_place = min(len(cells), len(chain))
    cap_dir = 1.0 if CAP_SIDE == "below" else -1.0

    print(f"PCB           : {PCB_PATH}")
    print(f"sensors       : {len(sensors)}   anchor(row,col) @ ({anchor[1]:.2f}, {anchor[0]:.2f})")
    print(f"LEDs (chain)  : {len(chain)}     LED-caps: {len(caps)}")
    print(f"grid cells    : {len(cells)}  (pitch {PITCH_X}x{PITCH_Y} mm)")
    print(f"placing       : {n_place} LEDs + {n_place} caps on board")
    print(f"parked        : {len(chain) - n_place} LEDs, {len(caps) - n_place} caps")
    if len(cells) > len(chain):
        print(f"NOTE: {len(cells) - len(chain)} grid cells left empty (fewer LEDs than cells)")

    # target position for every LED and cap
    targets = {}
    for i, ref in enumerate(chain):
        if i < n_place:
            targets[ref] = (cells[i][0], cells[i][1], LED_ROT)
        else:
            k = i - n_place
            targets[ref] = (PARK_X + k * PARK_STEP, PARK_Y, LED_ROT)
    for i, ref in enumerate(caps):
        if i < n_place:
            cx, cy = cells[i][0], cells[i][1] + cap_dir * CAP_OFFSET
            targets[ref] = (round(cx, 4), round(cy, 4), CAP_ROT)
        else:
            k = i - n_place
            targets[ref] = (PARK_X + k * PARK_STEP, PARK_Y + PARK_STEP, CAP_ROT)

    if args.preview:
        render_preview(fps, sensors, cells, chain, caps, n_place, cap_dir)

    if not args.apply:
        print("\n(dry run — re-run with --apply to write changes)")
        return

    # backup
    stamp = datetime.datetime.now().strftime("%Y%m%d-%H%M%S")
    bak = f"{PCB_PATH}.{stamp}.bak"
    open(bak, "w", encoding="utf-8").write(txt)
    print(f"\nbackup written: {bak}")

    # apply edits (reverse offset order so spans stay valid)
    edits = []
    for ref, (x, y, rot) in targets.items():
        s, e = fps[ref]["span"]
        b = set_position(txt[s:e], x, y, rot)
        edits.append((s, e, b))
    edits.sort(reverse=True)
    new = txt
    for s, e, b in edits:
        new = new[:s] + b + new[e:]

    if HIDE_ALL_LABELS:
        out = []; i = 0
        for s, e in footprint_spans(new):
            out.append(new[i:s]); out.append(hide_reference(new[s:e])); i = e
        out.append(new[i:]); new = "".join(out)

    assert new.count("(") == new.count(")"), "paren imbalance — aborting"
    open(PCB_PATH, "w", encoding="utf-8").write(new)
    hidden = new.count('(hide yes)')
    print(f"applied. footprints={new.count(chr(10)+chr(9)+'(footprint ')}  hidden-props={hidden}")


def render_preview(fps, sensors, cells, chain, caps, n_place, cap_dir):
    try:
        import matplotlib; matplotlib.use("Agg"); import matplotlib.pyplot as plt
    except ImportError:
        print("(matplotlib not installed — skipping preview)"); return
    fig, ax = plt.subplots(figsize=(18, 8))
    edge = BOARD_OUTLINE + [BOARD_OUTLINE[0]]
    ax.plot([p[0] for p in edge], [p[1] for p in edge], 'k-', lw=2)
    silk = SILK_POLY + [SILK_POLY[0]]
    ax.plot([p[0] for p in silk], [p[1] for p in silk], 'm--', lw=1)
    ax.scatter([s[0] for s in sensors], [s[1] for s in sensors], c='red', s=60, marker='x', zorder=6, label=f'sensor x{len(sensors)}')
    px = [c[0] for c in cells[:n_place]]; py = [c[1] for c in cells[:n_place]]
    ax.scatter(px, py, c='green', s=26, zorder=4, label=f'LED x{n_place}')
    ax.scatter(px, [y + cap_dir * CAP_OFFSET for y in py], c='deepskyblue', s=8, zorder=3, label=f'cap ({CAP_SIDE} 180°)' if CAP_ROT else f'cap ({CAP_SIDE})')
    ax.set_aspect('equal'); ax.invert_yaxis(); ax.legend(loc='lower right')
    ax.set_title(f"LED grid {PITCH_X}x{PITCH_Y}mm — rows anchored to sensor Y, around sensors, ignoring keycap holes")
    out = os.path.join(HERE, "led_grid_preview.png")
    plt.savefig(out, dpi=92, bbox_inches='tight'); print(f"preview: {out}")


if __name__ == "__main__":
    main()
