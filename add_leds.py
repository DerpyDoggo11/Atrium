#!/usr/bin/env python3
"""
Atrium PCB — add N new addressable SK6805 LEDs (+ 100nF caps) to BOTH the
schematic and the board, appended to the tail of the existing LED data chain.

Connectivity is done entirely with global labels placed on each pin
(+5V_LED / GND / chain-in / chain-out), so no wires are needed.  New PCB
footprints are created parked; run led_grid.py afterwards to place them.

    python add_leds.py -n 3          # add 3 (dry run: prints plan)
    python add_leds.py -n 106 --apply
"""
import os, re, math, argparse, datetime, uuid

HERE = os.path.dirname(os.path.abspath(__file__))
PCB_PATH = os.path.join(HERE, "Atrium PCB", "Atrium PCB.kicad_pcb")
SCH_PATH = os.path.join(HERE, "Atrium PCB", "Atrium PCB.kicad_sch")

ROOT_SHEET = "ee0455fb-9ea5-4694-a839-3916064005e2"   # flat-sheet instances path
CHAIN_TAIL_REF = "LED238"                              # current open DOUT
CHAIN_NET = "LEDFILL_{:03d}"                           # new chain net names
# LED pin offsets (rot 0, no mirror; lib is Y-up so pins Y-flip on instantiation)
LED_PINS = {"1": (-10.16, -1.27), "2": (-10.16, 1.27), "3": (10.16, 1.27), "4": (10.16, -1.27)}
LED_NET_PIN = {"vdd": "2", "gnd": "4", "din": "1", "dout": "3"}
CAP_PINS = {"1": (0.0, -3.81), "2": (0.0, 3.81)}       # pin1=+5V_LED, pin2=GND
# where to drop the new symbols in empty schematic space
SYM_ORIGIN_LED = (40.0, 600.0); SYM_DX, SYM_DY, SYM_COLS = 30.0, 22.0, 22
SYM_ORIGIN_CAP = (40.0, 980.0)


def newid(): return str(uuid.uuid4())

# ---------------- schematic symbol blocks ----------------
def sblocks(t):
    out = []; i = 0
    while True:
        j = t.find('\n\t(symbol\n', i)
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

def get_template(sch, lib_id, want_value=None):
    for s, e in sblocks(sch):
        b = sch[s:e]
        if f'(lib_id "{lib_id}")' in b and '(mirror' not in b:
            if want_value is None or re.search(rf'\(property "Value" "{re.escape(want_value)}"', b):
                return b
    raise RuntimeError("template not found")

def make_symbol(tmpl, ref, x, y, rot):
    """Clone a symbol block: new uuids, reference, shifted property positions, instances ref."""
    top_uuid = newid()
    tmat = re.search(r'\n\t\t\(at ([-0-9.]+) ([-0-9.]+)(?: [-0-9.]+)?\)', tmpl)
    tx, ty = float(tmat.group(1)), float(tmat.group(2))
    dx, dy = x - tx, y - ty
    b = tmpl
    b = re.sub(r'(\(uuid ")[0-9a-fA-F-]+(")', lambda m: m.group(1) + newid() + m.group(2), b)
    b = re.sub(r'(\(dnp \w+\)\n\t\t\(uuid ")[0-9a-fA-F-]+(")', r'\g<1>' + top_uuid + r'\g<2>', b, count=1)
    # shift every property (at PX PY R) by the symbol displacement (they are absolute sheet coords)
    def shift(m):
        return f'(at {float(m.group(1)) + dx:.4f} {float(m.group(2)) + dy:.4f} {m.group(3)})'
    b = re.sub(r'\(at ([-0-9.]+) ([-0-9.]+) ([-0-9.]+)\)', shift, b)
    # now set the symbol's own position (its (at) had no rotation field originally -> matched shift too;
    # force it explicitly)
    b = re.sub(r'\n\t\t\(at [-0-9.]+ [-0-9.]+ [-0-9.]+\)', f'\n\t\t(at {x} {y} {rot})', b, count=1)
    b = re.sub(r'(\(property "Reference" ")[^"]*(")', r'\g<1>' + ref + r'\g<2>', b, count=1)
    # instances reference is 5 tabs deep
    b = re.sub(r'(\(reference ")[^"]*(")', r'\g<1>' + ref + r'\g<2>', b, count=1)
    return b, top_uuid

def make_glabel(net, x, y, rot=0):
    return (f'\t(global_label "{net}"\n\t\t(shape input)\n\t\t(at {x} {y} {rot})\n'
            f'\t\t(effects\n\t\t\t(font\n\t\t\t\t(size 1.27 1.27)\n\t\t\t)\n\t\t\t(justify left)\n\t\t)\n'
            f'\t\t(uuid "{newid()}")\n'
            f'\t\t(property "Intersheetrefs" "${{INTERSHEET_REFS}}"\n\t\t\t(at {x} {y} 0)\n'
            f'\t\t\t(hide yes)\n\t\t\t(show_name no)\n\t\t\t(do_not_autoplace no)\n'
            f'\t\t\t(effects\n\t\t\t\t(font\n\t\t\t\t\t(size 1.27 1.27)\n\t\t\t\t)\n\t\t\t)\n\t\t)\n\t)\n')

# ---------------- PCB footprints ----------------
def fp_spans(txt):
    out = []; i = 0
    while True:
        j = txt.find('\n\t(footprint ', i)
        if j < 0: break
        k = j + 1; d = 0; st = False
        while k < len(txt):
            c = txt[k]
            if c == '(': d += 1; st = True
            elif c == ')':
                d -= 1
                if d == 0 and st: out.append((j + 1, k + 1)); break
            k += 1
        i = k + 1
    return out

def fp_by_ref(txt, want):
    for s, e in fp_spans(txt):
        b = txt[s:e]
        m = re.search(r'\(property "Reference" "([^"]+)"', b)
        if m and m.group(1) == want:
            return s, e, b
    return None

def fp_template(txt, value):
    for s, e in fp_spans(txt):
        b = txt[s:e]
        if re.search(r'\(property "Value" "%s"' % re.escape(value), b) and \
           re.search(r'\(property "Reference" "(LED|C)\d+"', b):
            return b
    raise RuntimeError("fp template not found")

def make_footprint(tmpl, ref, sym_uuid, x, y, padnets):
    b = re.sub(r'(\(uuid ")[0-9a-fA-F-]+(")', lambda m: m.group(1) + newid() + m.group(2), b := tmpl)
    b = re.sub(r'(\(property "Reference" ")[^"]*(")', r'\g<1>' + ref + r'\g<2>', b, count=1)
    b = re.sub(r'\n\t\t\(at [-0-9.]+ [-0-9.]+(?: [-0-9.]+)?\)', f'\n\t\t(at {x} {y})', b, count=1)
    b = re.sub(r'\(path "[^"]*"\)', f'(path "/{sym_uuid}")', b, count=1)
    for pad, net in padnets.items():
        pat = re.compile(r'(\(pad "' + pad + r'"[^\n]*(?:\n(?!\s*\(pad ).*)*?\(net ")[^"]*(")')
        b, n = pat.subn(lambda m: m.group(1) + net + m.group(2), b, count=1)
        assert n == 1, f"pad {pad} net"
    return b

def set_pad_net(block, pad, net):
    pat = re.compile(r'(\(pad "' + pad + r'"[^\n]*(?:\n(?!\s*\(pad ).*)*?\(net ")[^"]*(")')
    b, n = pat.subn(lambda m: m.group(1) + net + m.group(2), block, count=1)
    assert n == 1
    return b


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("-n", type=int, required=True)
    ap.add_argument("--apply", action="store_true")
    args = ap.parse_args()
    N = args.n

    sch = open(SCH_PATH, encoding="utf-8").read()
    pcb = open(PCB_PATH, encoding="utf-8").read()

    led_tmpl_s = get_template(sch, "easyeda2kicad:SK6805-EC15")
    cap_tmpl_s = get_template(sch, "Device:C", "100nF")
    led_tmpl_p = fp_template(pcb, "SK6805-EC15")
    cap_tmpl_p = fp_template(pcb, "100nF")

    max_led = max(int(m) for m in re.findall(r'\(property "Reference" "LED(\d+)"', sch))
    max_cap = max(int(m) for m in re.findall(r'\(property "Reference" "C(\d+)"', sch))
    led_refs = [f"LED{max_led + 1 + i}" for i in range(N)]
    cap_refs = [f"C{max_cap + 1 + i}" for i in range(N)]

    # LED238 DOUT pin (schematic, rot 0) -> attach chain head
    s238 = None
    for s, e in sblocks(sch):
        b = sch[s:e]
        if '(lib_id "easyeda2kicad:SK6805-EC15")' in b and f'(reference "{CHAIN_TAIL_REF}")' in b:
            at = re.search(r'\n\t\t\(at ([-0-9.]+) ([-0-9.]+)(?: ([-0-9.]+))?\)', b)
            s238 = (float(at.group(1)), float(at.group(2)), float(at.group(3) or 0)); break
    assert s238 and s238[2] == 0, "LED238 not found / rotated"
    tail_dout = (s238[0] + LED_PINS["3"][0], s238[1] + LED_PINS["3"][1])

    print(f"adding {N} LEDs ({led_refs[0]}..{led_refs[-1]}) + {N} caps ({cap_refs[0]}..{cap_refs[-1]})")
    print(f"chain: {CHAIN_TAIL_REF}.DOUT -> {CHAIN_NET.format(0)} -> ... (labels at pins)")

    add_sch = ""; add_fp = ""
    # chain-head label on LED238 DOUT
    add_sch += make_glabel(CHAIN_NET.format(0), tail_dout[0], tail_dout[1])

    for i in range(N):
        col = i % SYM_COLS; row = i // SYM_COLS
        ox = SYM_ORIGIN_LED[0] + col * SYM_DX; oy = SYM_ORIGIN_LED[1] + row * SYM_DY
        din = CHAIN_NET.format(i); dout = CHAIN_NET.format(i + 1)
        sym, uid = make_symbol(led_tmpl_s, led_refs[i], ox, oy, 0)
        add_sch += "\t" + sym.strip() + "\n"
        add_sch += make_glabel("+5V_LED", ox + LED_PINS["2"][0], oy + LED_PINS["2"][1])
        add_sch += make_glabel("GND",     ox + LED_PINS["4"][0], oy + LED_PINS["4"][1])
        add_sch += make_glabel(din,       ox + LED_PINS["1"][0], oy + LED_PINS["1"][1])
        if i < N - 1:
            add_sch += make_glabel(dout,  ox + LED_PINS["3"][0], oy + LED_PINS["3"][1])
        add_fp += "\t" + make_footprint(led_tmpl_p, led_refs[i], uid, SYM_ORIGIN_LED[0] + i,
                                        1400.0,
                                        {"1": din, "2": "+5V_LED", "3": dout if i < N - 1 else f"unconnected-({led_refs[i]}-DOUT-Pad3)", "4": "GND"}).strip() + "\n"

    for i in range(N):
        col = i % SYM_COLS; row = i // SYM_COLS
        ox = SYM_ORIGIN_CAP[0] + col * SYM_DX; oy = SYM_ORIGIN_CAP[1] + row * SYM_DY
        sym, uid = make_symbol(cap_tmpl_s, cap_refs[i], ox, oy, 0)
        add_sch += "\t" + sym.strip() + "\n"
        add_sch += make_glabel("+5V_LED", ox + CAP_PINS["1"][0], oy + CAP_PINS["1"][1])
        add_sch += make_glabel("GND",     ox + CAP_PINS["2"][0], oy + CAP_PINS["2"][1])
        add_fp += "\t" + make_footprint(cap_tmpl_p, cap_refs[i], uid, SYM_ORIGIN_CAP[0] + i, 1420.0,
                                        {"1": "+5V_LED", "2": "GND"}).strip() + "\n"

    if not args.apply:
        print(f"\n(dry run) would add {add_sch.count('(symbol')} symbols, "
              f"{add_sch.count('(global_label')} labels, {add_fp.count('(footprint ')} footprints")
        return

    stamp = datetime.datetime.now().strftime("%Y%m%d-%H%M%S")
    open(f"{SCH_PATH}.{stamp}.bak", "w", encoding="utf-8").write(sch)
    open(f"{PCB_PATH}.{stamp}.bak", "w", encoding="utf-8").write(pcb)

    # schematic: retarget LED238 DOUT already handled by label; insert new content before final ')'
    k = sch.rstrip().rfind(")")
    sch = sch[:k] + add_sch + sch[k:]
    open(SCH_PATH, "w", encoding="utf-8").write(sch)

    # pcb: retarget LED238 pad3 to chain head, insert new footprints
    s, e, b = fp_by_ref(pcb, CHAIN_TAIL_REF)
    pcb = pcb[:s] + set_pad_net(b, "3", CHAIN_NET.format(0)) + pcb[e:]
    k = pcb.find('\n\t(footprint ')
    kk = k + 1; d = 0; st = False
    while kk < len(pcb):
        c = pcb[kk]
        if c == '(': d += 1; st = True
        elif c == ')':
            d -= 1
            if d == 0 and st: break
        kk += 1
    pcb = pcb[:kk + 1] + "\n" + add_fp.rstrip("\n") + pcb[kk + 1:]
    open(PCB_PATH, "w", encoding="utf-8").write(pcb)
    assert sch.count("(") == sch.count(")"), "sch paren imbalance"
    assert pcb.count("(") == pcb.count(")"), "pcb paren imbalance"
    print(f"applied. backups .{stamp}.bak")


if __name__ == "__main__":
    main()
