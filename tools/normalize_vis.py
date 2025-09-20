#!/usr/bin/env python3
import sys, re

pat = re.compile(r'^(\d+):(.*)$')
def parse_pts(s):
    s = s.strip()
    if s.endswith(','): s = s[:-1]
    pts = []
    for tok in s.split(','):
        tok = tok.strip()
        if not tok: continue
        m = re.match(r'^\(?\s*(\d+)\s*,\s*(\d+)\s*\)?$', tok)
        if not m: continue
        pts.append((int(m.group(1)), int(m.group(2))))
    return pts

def fmt_line(i, pts):
    return f"{i}:" + ",".join(f"({x},{y})" for x,y in pts)

lines=[]
with open(sys.argv[1],'r',encoding='utf-8',errors='ignore') as f:
    for ln in f:
        m = pat.match(ln.strip())
        if not m: continue
        idx = int(m.group(1))
        pts = parse_pts(m.group(2))
        lines.append((idx, pts))

if not lines:
    print("[ERR] no agent lines found", file=sys.stderr); sys.exit(2)

T = max(len(pts) for _,pts in lines)
out = []
for idx,pts in lines:
    if not pts:
        print(f"[WARN] agent {idx} empty path; fill with (0,0)")
        pts = [(0,0)]
    if len(pts) < T:
        pts = pts + [pts[-1]]*(T - len(pts))
    out.append((idx, pts))

out.sort(key=lambda x: x[0])
with open(sys.argv[2],'w',encoding='utf-8') as g:
    for idx, pts in out:
        g.write(fmt_line(idx, pts) + "\n")
print(f"[OK] normalized to {T} steps across {len(out)} agents -> {sys.argv[2]}")
