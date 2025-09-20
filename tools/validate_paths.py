#!/usr/bin/env python3
import argparse, re, sys
PASSABLE = {'.','S','G','_'}
ap = argparse.ArgumentParser()
ap.add_argument("--map", required=True)
ap.add_argument("--paths", required=True)  # 元の EECBS の paths（Agent i: ...）
args = ap.parse_args()

# map 読み込み（MovingAI）
with open(args.map) as f:
    lines = [l.rstrip('\n') for l in f]
def get_dim(key):
    for l in lines:
        t=l.strip().split()
        if len(t)>=2 and t[0].lower()==key: return int(t[1])
    print(f"[ERR] {key} not found", file=sys.stderr); sys.exit(1)
H = get_dim("height"); W = get_dim("width")
try:
    start = lines.index("map")+1
except ValueError:
    print("[ERR] 'map' section not found", file=sys.stderr); sys.exit(1)
grid = lines[start:start+H]
if len(grid)!=H or any(len(row)!=W for row in grid):
    print("[ERR] map size mismatch", file=sys.stderr); sys.exit(1)
def passable(x,y):
    if not (0<=x<W and 0<=y<H): return False
    return grid[y][x] in PASSABLE

agent_re = re.compile(r"^\s*Agent\s+(\d+):")
coord_re = re.compile(r"\((\d+)\s*,\s*(\d+)\)")
bad=0
with open(args.paths) as f:
    for line in f:
        m = agent_re.match(line)
        if not m: continue
        aid=int(m.group(1))
        coords=[(int(a),int(b)) for a,b in coord_re.findall(line)]
        for t,(x,y) in enumerate(coords):
            if not passable(x,y):
                print(f"[BAD] Agent {aid} t={t} on obstacle/out-of-range at ({x},{y})")
                bad+=1
        for t in range(len(coords)-1):
            x1,y1=coords[t]; x2,y2=coords[t+1]
            if abs(x1-x2)+abs(y1-y2)!=1:
                print(f"[BAD] Agent {aid} step {t}->{t+1} not 4-neighbor: ({x1},{y1})->({x2},{y2})")
                bad+=1
print("[OK] all points passable & 4-neighbor moves" if bad==0 else f"[NG] found {bad} issues")
