"""Crop LOGO texture footprints to the board outline, and cut a margin
rectangle around the ATRIUM wordmark in the bottom texture.

Usage: python crop_texture.py [--apply]
Without --apply: writes cropped board to scratchpad + a render, does not touch project.
With --apply:    backs up and overwrites the project .kicad_pcb.
"""
import re, sys, math, uuid, os, shutil

PCB = r"C:\Users\vrs18\OneDrive\Desktop\Atrium\Atrium PCB\Atrium PCB.kicad_pcb"
ATRIUM_MARGIN = 4.0  # mm around ATRIUM text bbox

# footprint origins (identify targets)
TOP_TEX = (129.37588, 49.99)       # F.Cu + F.SilkS
BOT_TEX = (129.153931, 50.047081)  # B.Cu + B.SilkS
ATRIUM  = (136.489653, 57.826873)  # wordmark, leave intact

from shapely.geometry import Polygon, MultiPolygon, box
from shapely.ops import unary_union, polygonize
from shapely import make_valid

t = open(PCB, encoding="utf-8").read()

# ---- build board polygon from Edge.Cuts (exclude stray notch line L0) ----
def bez(p, n=48):
    o = []
    for i in range(n + 1):
        u = i / n; mt = 1 - u
        o.append((mt**3*p[0][0]+3*mt*mt*u*p[1][0]+3*mt*u*u*p[2][0]+u**3*p[3][0],
                  mt**3*p[0][1]+3*mt*mt*u*p[1][1]+3*mt*u*u*p[2][1]+u**3*p[3][1]))
    return o

subsegs = []
for m in re.finditer(r"\(gr_line\b", t):
    s = t[m.start():m.start()+400]
    if "Edge.Cuts" not in s: continue
    a = re.search(r"\(start (-?[\d.]+) (-?[\d.]+)\).*?\(end (-?[\d.]+) (-?[\d.]+)\)", s, re.S)
    x1,y1,x2,y2 = map(float, a.groups())
    # skip stray internal notch line: nearly horizontal around y~33, well inside board
    if abs(y1-y2) < 2 and 30 < (y1+y2)/2 < 36 and min(x1,x2) > 40:
        continue
    subsegs.append([(x1,y1),(x2,y2)])
for m in re.finditer(r"\(gr_curve\b", t):
    s = t[m.start():m.start()+500]
    if "Edge.Cuts" not in s: continue
    pts = [(float(x),float(y)) for x,y in re.findall(r"\(xy (-?[\d.]+) (-?[\d.]+)\)", s)]
    bp = bez(pts)
    for i in range(len(bp)-1):
        subsegs.append([bp[i], bp[i+1]])

# Board is convex (flat top, vertical sides, V-trapezoid bottom). The stored
# Edge.Cuts has ~2.5mm gaps and overlaps at the bottom tip, so polygonize is
# unreliable; the convex hull of all perimeter points is exact for this shape.
from shapely.geometry import MultiPoint, LineString
allpts = [p for seg in subsegs for p in seg]
board = make_valid(MultiPoint(allpts).convex_hull)
# sanity: hull must be a Polygon covering the board bbox
assert board.geom_type == "Polygon", board.geom_type
print("board area=%.1f bounds=%s hull_pts=%d"
      % (board.area, [round(v,2) for v in board.bounds], len(board.exterior.coords)))

# ---- geometry transforms (KiCad footprint item, CW rot in Y-down) ----
def fwd(x, y, ox, oy, th):
    c, s = math.cos(th), math.sin(th)
    return (ox + x*c + y*s, oy - x*s + y*c)
def inv(ax, ay, ox, oy, th):
    c, s = math.cos(th), math.sin(th)
    dx, dy = ax-ox, ay-oy
    return (c*dx - s*dy, s*dx + c*dy)

def parse_at(seg):
    m = re.search(r"\n\t\t\(at (-?[\d.]+) (-?[\d.]+)(?: (-?[\d.]+))?", seg)
    return float(m.group(1)), float(m.group(2)), float(m.group(3) or 0)

def iter_fp_poly(seg):
    """yield (start,end,layer,pts) for each fp_poly in footprint text."""
    starts = [m.start() for m in re.finditer(r"\(fp_poly\b", seg)]
    starts.append(len(seg))
    for i in range(len(starts)-1):
        sub = seg[starts[i]:starts[i+1]]
        ly = re.search(r'\(layer "([^"]+)"', sub)
        pts = [(float(a),float(b)) for a,b in re.findall(r"\(xy (-?[\d.]+) (-?[\d.]+)\)", sub)]
        yield starts[i], starts[i+1], (ly.group(1) if ly else None), pts

def emit_poly(ring_local, layer):
    body = " ".join("(xy %.6f %.6f)" % (x,y) for x,y in ring_local)
    return ('\t\t(fp_poly\n\t\t\t(pts\n\t\t\t\t%s\n\t\t\t)\n'
            '\t\t\t(stroke\n\t\t\t\t(width 0)\n\t\t\t\t(type solid)\n\t\t\t)\n'
            '\t\t\t(fill yes)\n\t\t\t(layer "%s")\n\t\t\t(uuid "%s")\n\t\t)\n'
            % (body, layer, uuid.uuid4()))

def open_holes(poly):
    """KiCad fp_poly has no holes: cut a full-height hairline slit through each
    interior ring so every hole opens to the outside. One pass, no recursion."""
    if not list(poly.interiors):
        return [poly]
    minx, miny, maxx, maxy = poly.bounds
    cuts = []
    for ring in poly.interiors:
        p = Polygon(ring).representative_point()  # guaranteed inside the hole
        cuts.append(box(p.x - 0.005, miny - 1.0, p.x + 0.005, maxy + 1.0))
    res = poly.difference(unary_union(cuts))
    out = []
    for g in (res.geoms if hasattr(res, "geoms") else [res]):
        if g.geom_type == "Polygon" and g.area > 0:
            out.append(Polygon(g.exterior))  # drop any sliver hole remnants
    return out

hole_count = 0
def clip_footprint(seg, region):
    """Rebuild all fp_poly in seg, clipped to region (shapely)."""
    global hole_count
    ox, oy, rot = parse_at(seg)
    th = math.radians(rot)
    blocks = list(iter_fp_poly(seg))
    if not blocks:
        return seg, 0, 0
    kept = []
    nin = len(blocks)
    for s0, s1, layer, pts in blocks:
        if layer is None or len(pts) < 3:
            continue
        absr = [fwd(x, y, ox, oy, th) for x, y in pts]
        try:
            poly = make_valid(Polygon(absr))
        except Exception:
            continue
        inter = poly.intersection(region)
        if inter.is_empty:
            continue
        geoms = inter.geoms if hasattr(inter, "geoms") else [inter]
        for g in geoms:
            if g.geom_type != "Polygon" or g.area <= 0:
                continue
            for simple in open_holes(g):
                if list(simple.interiors):
                    hole_count += 1  # should be 0 now
                ring = [inv(ax, ay, ox, oy, th) for ax, ay in simple.exterior.coords[:-1]]
                kept.append(emit_poly(ring, layer))
    # splice: replace the contiguous fp_poly run only. End of run = matching
    # close paren of the LAST fp_poly, so the footprint tail/close is preserved.
    first = seg.rfind("\n", 0, blocks[0][0]) + 1  # start of first fp_poly's line
    last_start = blocks[-1][0]
    d = 0; j = last_start
    while j < len(seg):
        if seg[j] == "(":
            d += 1
        elif seg[j] == ")":
            d -= 1
            if d == 0:
                break
        j += 1
    last_end = j + 1  # just past the last fp_poly's closing ')'
    new_seg = seg[:first] + "".join(kept) + seg[last_end:]
    return new_seg, nin, len(kept)

# ---- walk footprints, apply ----
fps = [m.start() for m in re.finditer(r"\n\t\(footprint ", t)]; fps.append(len(t))
atr_rect = box(ATRIUM[0], 0, 0, 0)  # placeholder, real one below
# compute ATRIUM abs bbox
for i in range(len(fps)-1):
    seg = t[fps[i]:fps[i+1]]
    if '"LOGO"' not in seg: continue
    ox, oy, rot = parse_at(seg)
    if abs(ox-ATRIUM[0]) < .01 and abs(oy-ATRIUM[1]) < .01:
        th = math.radians(rot)
        xs=[];ys=[]
        for _,_,layer,pts in iter_fp_poly(seg):
            for x,y in pts:
                ax,ay = fwd(x,y,ox,oy,th); xs.append(ax); ys.append(ay)
        atr_rect = box(min(xs)-ATRIUM_MARGIN, min(ys)-ATRIUM_MARGIN,
                       max(xs)+ATRIUM_MARGIN, max(ys)+ATRIUM_MARGIN)
        print("ATRIUM bbox x[%.2f,%.2f] y[%.2f,%.2f] -> cut rect %s"
              % (min(xs),max(xs),min(ys),max(ys), [round(v,2) for v in atr_rect.bounds]))

board_minus_atrium = board.difference(atr_rect)

out = []
pos = 0
for i in range(len(fps)-1):
    seg = t[fps[i]:fps[i+1]]
    if '"LOGO"' in seg:
        ox, oy, rot = parse_at(seg)
        region = None
        if abs(ox-TOP_TEX[0]) < .01 and abs(oy-TOP_TEX[1]) < .01:
            region = board; tag="TOP texture"
        elif abs(ox-BOT_TEX[0]) < .01 and abs(oy-BOT_TEX[1]) < .01:
            region = board_minus_atrium; tag="BOT texture"
        if region is not None:
            new_seg, nin, nout = clip_footprint(seg, region)
            print("  %s: %d -> %d fp_poly" % (tag, nin, nout))
            out.append((fps[i], fps[i+1], new_seg))

# rebuild full text
res = []
last = 0
for a, b, ns in out:
    res.append(t[last:a]); res.append(ns); last = b
res.append(t[last:])
newt = "".join(res)
print("polys with holes (need manual check):", hole_count)

apply = "--apply" in sys.argv
if apply:
    bak = PCB + ".pretex.bak"
    if not os.path.exists(bak):
        shutil.copy(PCB, bak)
    open(PCB, "w", encoding="utf-8").write(newt)
    print("APPLIED to project. backup:", bak)
else:
    sp = os.environ.get("SP", ".")
    op = os.path.join(sp, "Atrium PCB.kicad_pcb")
    open(op, "w", encoding="utf-8").write(newt)
    print("preview written:", op)
