#!/usr/bin/env python3
"""
Complete the MAX98357A (U48) I2S class-D speaker circuit for the Atrium board.

Adds, in BOTH schematic and PCB:
  - I2S:  DIN<->RP2350 GPIO4, BCLK<->GPIO5, LRCLK<->GPIO6   (new nets I2S_*)
  - GAIN_SLOT -> GND (9 dB)   and   thermal PAD -> GND
  - VDD decoupling: 100nF (0603) + 10uF (0805) on +3.3V/GND next to U48
  - places U48 + the two caps just below the speaker SPK1
(OUTP/OUTN -> SPK1 and VDD/GND are already wired.)

    python add_speaker.py            # dry run
    python add_speaker.py --apply
"""
import os, re, argparse, datetime
import add_leds as A          # reuse tested helpers

SCH = A.SCH_PATH; PCB = A.PCB_PATH

# --- pin connection points (schematic sheet coords) ---
# U48 MAX98357A at (434.34,204.47) rot0; lib is Y-up so pins Y-flip on instantiation.
U48 = (434.34, 204.47)
U48_PINS = {"DIN": (U48[0]-12.7, U48[1]-7.62), "BCLK": (U48[0]-12.7, U48[1]-5.08),
            "LRCLK": (U48[0]-12.7, U48[1]-2.54), "GAIN": (U48[0]-12.7, U48[1]+5.08),
            "PAD": (U48[0]+2.54, U48[1]+12.7)}
# RP2350 right-side GPIO labels live at x=187.96, GPIO_n at y=57.15+n*2.54
RP_GPIO = {n: (187.96, 57.15 + n*2.54) for n in (4, 5, 6)}

I2S = {"I2S_DIN":  ("DIN",  4), "I2S_BCLK": ("BCLK", 5), "I2S_LRCLK": ("LRCLK", 6)}
# PCB pad numbers on U48 for each signal
U48_PAD = {"DIN": "1", "BCLK": "16", "LRCLK": "14", "GAIN": "2", "PAD": "17"}
RP_PAD = {4: "7", 5: "8", 6: "9"}

# placement (PCB) — audio cluster just below the 23mm speaker at (129.4,39.6)
U48_XY = (129.4, 61.0)
C100_XY = (121.5, 61.0)
C10_XY = (137.3, 61.0)
# schematic placement for the two new cap symbols (empty area)
C100_SXY = (90.0, 620.0)
C10_SXY = (110.0, 620.0)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--apply", action="store_true")
    args = ap.parse_args()
    sch = open(SCH, encoding="utf-8").read()
    pcb = open(PCB, encoding="utf-8").read()

    cap100_s = A.get_template(sch, "Device:C", "100nF")
    cap10_s = A.get_template(sch, "Device:C", "10uF")
    cap100_p = A.fp_template(pcb, "100nF")
    cap10_p = A.fp_template(pcb, "10uF")
    max_c = max(int(m) for m in re.findall(r'\(property "Reference" "C(\d+)"', sch))
    c100, c10 = f"C{max_c+1}", f"C{max_c+2}"

    # ---------- schematic additions ----------
    add = ""
    for net, (pin, gpio) in I2S.items():
        add += A.make_glabel(net, *U48_PINS[pin])
        add += A.make_glabel(net, *RP_GPIO[gpio])
    add += A.make_glabel("GND", *U48_PINS["GAIN"])
    add += A.make_glabel("GND", *U48_PINS["PAD"])
    # decoupling caps (pin1=+3.3V at Oy-3.81, pin2=GND at Oy+3.81)
    for ref, tmpl, (ox, oy) in ((c100, cap100_s, C100_SXY), (c10, cap10_s, C10_SXY)):
        sym, uid = A.make_symbol(tmpl, ref, ox, oy, 0)
        add += "\t" + sym.strip() + "\n"
        add += A.make_glabel("+3.3V", ox, oy - 3.81)
        add += A.make_glabel("GND", ox, oy + 3.81)
        globals()[ref + "_uid"] = uid

    # ---------- pcb additions ----------
    def setnet(txt, ref, pad, net):
        r = A.fp_by_ref(txt, ref)
        assert r, f"{ref} not found"
        s, e, b = r
        return txt[:s] + A.set_pad_net(b, pad, net) + txt[e:]
    for net, (pin, gpio) in I2S.items():
        pcb = setnet(pcb, "U48", U48_PAD[pin], net)
        pcb = setnet(pcb, "RP2350", RP_PAD[gpio], net)
    pcb = setnet(pcb, "U48", U48_PAD["GAIN"], "GND")
    pcb = setnet(pcb, "U48", U48_PAD["PAD"], "GND")
    # move U48 onto the board
    s, e, b = A.fp_by_ref(pcb, "U48")
    b = re.sub(r'\n\t\t\(at [-0-9.]+ [-0-9.]+(?: [-0-9.]+)?\)', f'\n\t\t(at {U48_XY[0]} {U48_XY[1]})', b, count=1)
    pcb = pcb[:s] + b + pcb[e:]
    # new cap footprints
    add_fp = ""
    add_fp += "\t" + A.make_footprint(cap100_p, c100, globals()[c100+"_uid"], *C100_XY,
                                      {"1": "+3.3V", "2": "GND"}).strip() + "\n"
    add_fp += "\t" + A.make_footprint(cap10_p, c10, globals()[c10+"_uid"], *C10_XY,
                                      {"1": "+3.3V", "2": "GND"}).strip() + "\n"

    print("I2S: RP2350 GPIO4/5/6 <-> U48 DIN/BCLK/LRCLK ; GAIN,PAD -> GND")
    print(f"decoupling caps: {c100} (100nF) + {c10} (10uF)")
    print(f"U48 placed at {U48_XY}; caps at {C100_XY}/{C10_XY}")
    if not args.apply:
        print("\n(dry run)"); return

    stamp = datetime.datetime.now().strftime("%Y%m%d-%H%M%S")
    open(f"{SCH}.{stamp}.bak", "w", encoding="utf-8").write(open(SCH, encoding="utf-8").read())
    open(f"{PCB}.{stamp}.bak", "w", encoding="utf-8").write(open(PCB, encoding="utf-8").read())
    k = sch.rstrip().rfind(")")
    sch2 = sch[:k] + add + sch[k:]
    # insert footprints after first footprint
    k = pcb.find('\n\t(footprint '); kk = k+1; d = 0; st = False
    while kk < len(pcb):
        c = pcb[kk]
        if c == '(': d += 1; st = True
        elif c == ')':
            d -= 1
            if d == 0 and st: break
        kk += 1
    pcb2 = pcb[:kk+1] + "\n" + add_fp.rstrip("\n") + pcb[kk+1:]
    assert sch2.count("(") == sch2.count(")"), "sch paren"
    assert pcb2.count("(") == pcb2.count(")"), "pcb paren"
    open(SCH, "w", encoding="utf-8").write(sch2)
    open(PCB, "w", encoding="utf-8").write(pcb2)
    print(f"applied. backups .{stamp}.bak")


if __name__ == "__main__":
    main()
