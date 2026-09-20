#!/usr/bin/env python3
"""Delete the LEDs/caps that don't fit on the board (the ones led_grid.py parks:
chain-tail LEDs + highest-ref caps) from BOTH the schematic and the PCB.

Safe delete: removes the symbol blocks (schematic) + footprint blocks (PCB) + the
+5V_LED global labels sitting on the deleted power pins. Data/GND wire stubs are
left in place (harmless dangling ends — clear them in KiCad with "Cleanup Wires"
if wanted). Removing pins can only ever DISCONNECT, never create a wrong short, so
the kept LEDs' nets are unchanged.

    python delete_parked.py           # dry run
    python delete_parked.py --apply   # write changes (timestamped backups first)
"""
import os, re, math, argparse, datetime, sys
import led_grid as L

SCH_PATH = os.path.join(L.HERE, "Atrium PCB", "Atrium PCB.kicad_sch")
PCB_PATH = L.PCB_PATH
TOL = 0.05
# power pin (holds a +5V_LED global label) offset for rot 0 — LED pin2, cap pin1
LED_VDD = (-10.16, 1.27)
CAP_VP  = (0.0, -3.81)


def rotcw(vx, vy, deg):
    r = math.radians(deg); c, s = math.cos(r), math.sin(r)
    return (vx * c + vy * s, -vx * s + vy * c)


def blocks(t, opener):
    out = []; i = 0
    while True:
        j = t.find(opener, i)
        if j < 0: break
        k = j + 1; d = 0; st = False
        while k < len(t):
            c = t[k]
            if c == '(': d += 1; st = True
            elif c == ')':
                d -= 1
                if d == 0 and st: out.append((j + 1, k + 1)); break
            k += 1
        i = k + 1
    return out


def parked_set():
    txt = open(PCB_PATH, encoding="utf-8").read()
    fps = L.parse_pcb(txt)
    sensors = [(f["x"], f["y"]) for f in fps.values() if f["value"] == L.SENSOR_VALUE]
    sensor_caps = [(f["x"], f["y"]) for r, f in fps.items()
                   if f["value"] == L.CAP_VALUE and set(f["nets"].values()) == {"+3.3V", "GND"}
                   and any(math.hypot(f["x"] - sx, f["y"] - sy) < 6 for sx, sy in sensors)]
    chain = L.led_chain_order(fps); caps = L.led_caps(fps)
    obstacles = L.component_obstacles(txt, fps, sensors)
    cells, _ = L.build_grid(sensors, sensor_caps, obstacles)
    n = min(len(cells), len(chain))
    return set(chain[n:]), set(caps[n:]), n


def main():
    ap = argparse.ArgumentParser(); ap.add_argument("--apply", action="store_true")
    args = ap.parse_args()

    parked_leds, parked_caps, n = parked_set()
    to_del = parked_leds | parked_caps
    print(f"on-board: {n} LEDs   deleting: {len(parked_leds)} LEDs + {len(parked_caps)} caps")

    sch = open(SCH_PATH, encoding="utf-8").read()

    # index +5V_LED labels by position
    v5_labels = {}
    for s, e in blocks(sch, '\n\t(global_label '):
        b = sch[s:e]
        if not b.startswith('\t(global_label "+5V_LED"'):
            continue
        at = re.search(r'\n\t\t\(at ([-0-9.]+) ([-0-9.]+)', b)
        v5_labels[(round(float(at.group(1)) / TOL), round(float(at.group(2)) / TOL))] = (s, e)

    def find_v5(x, y):
        kx, ky = round(x / TOL), round(y / TOL)
        for dx in (-1, 0, 1):
            for dy in (-1, 0, 1):
                sp = v5_labels.get((kx + dx, ky + dy))
                if sp: return sp
        return None

    del_sym = []; del_lbl = set(); found_pins = 0
    for s, e in blocks(sch, '\n\t(symbol\n'):
        b = sch[s:e]
        m = re.search(r'\(reference "([^"]+)"', b)
        if not m or m.group(1) not in to_del: continue
        ref = m.group(1)
        at = re.search(r'\n\t\t\(at ([-0-9.]+) ([-0-9.]+)(?: ([-0-9.]+))?\)', b)
        ox, oy, rot = float(at.group(1)), float(at.group(2)), float(at.group(3) or 0)
        vx, vy = rotcw(*(LED_VDD if ref in parked_leds else CAP_VP), rot)
        sp = find_v5(ox + vx, oy + vy)
        if sp: del_lbl.add(sp); found_pins += 1
        del_sym.append((s, e, ref))

    pcb = open(PCB_PATH, encoding="utf-8").read()
    fp_del = [(s, e) for s, e in L.footprint_spans(pcb) if L.ref_of(pcb[s:e]) in to_del]

    print(f"symbols to delete: {len(del_sym)}   +5V_LED labels: {len(del_lbl)} (pins matched {found_pins})")
    print(f"footprints to delete: {len(fp_del)}")
    if len(del_sym) != len(to_del) or len(fp_del) != len(to_del):
        print(f"ABORT: count mismatch (want {len(to_del)} each)"); sys.exit(1)
    # +5V_LED label cleanup is best-effort (an orphan pin may have no label)
    if found_pins < len(to_del) - 2:
        print(f"ABORT: only matched {found_pins}/{len(to_del)} +5V_LED labels"); sys.exit(1)

    if not args.apply:
        print("\n(dry run — re-run with --apply to write changes)"); return

    stamp = datetime.datetime.now().strftime("%Y%m%d-%H%M%S")
    open(f"{SCH_PATH}.{stamp}.bak", "w", encoding="utf-8").write(sch)
    open(f"{PCB_PATH}.{stamp}.bak", "w", encoding="utf-8").write(pcb)

    for s, e in sorted([(s, e) for s, e, _ in del_sym] + list(del_lbl), reverse=True):
        sch = sch[:s - 1] + sch[e:] if sch[s - 1] == '\n' else sch[:s] + sch[e:]
    for s, e in sorted(fp_del, reverse=True):
        pcb = pcb[:s - 1] + pcb[e:] if pcb[s - 1] == '\n' else pcb[:s] + pcb[e:]

    assert sch.count("(") == sch.count(")"), "sch paren imbalance"
    assert pcb.count("(") == pcb.count(")"), "pcb paren imbalance"
    open(SCH_PATH, "w", encoding="utf-8").write(sch)
    open(PCB_PATH, "w", encoding="utf-8").write(pcb)
    print(f"applied. backups .{stamp}.bak")


if __name__ == "__main__":
    main()
