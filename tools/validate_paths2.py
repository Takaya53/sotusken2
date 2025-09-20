#!/usr/bin/env python3
import argparse, re, sys

def load_movingai_map(path, obstacles='@'):
    H=W=None
    grid=[]
    with open(path,'r',encoding='utf-8',errors='ignore') as f:
        lines=[ln.rstrip('\n') for ln in f]
    # 基本ヘッダ: type, height, width, map
    for i,ln in enumerate(lines):
        if ln.strip().lower().startswith('height'):
            H=int(ln.split()[-1]); 
        if ln.strip().lower().startswith('width'):
            W=int(ln.split()[-1]);
        if ln.strip().lower()=='map':
            if H is None or W is None:
                print("[ERR] header has no height/width", file=sys.stderr); sys.exit(2)
            grid=lines[i+1:i+1+H]
            break
    if len(grid)!=H or any(len(r)!=W for r in grid):
        print(f"[ERR] grid size mismatch H={H} W={W} real=({len(grid)} x {len(grid[0]) if grid else 0})", file=sys.stderr); sys.exit(2)
    obs=set(obstacles)
    passable=[[ (c not in obs) for c in row ] for row in grid]
    return H,W,passable

pat_path_line=re.compile(r'^Agent\s+(\d+):\s*(.*)$')
pat_point=re.compile(r'\((\d+),\s*(\d+)\)')

def load_eecbs_paths(path):
    agents=[]
    with open(path,'r',encoding='utf-8',errors='ignore') as f:
        for ln in f:
            m=pat_path_line.match(ln.strip())
            if not m: 
                continue
            seq=[]
            for x,y in pat_point.findall(m.group(2)):
                seq.append( (int(x),int(y)) )
            agents.append(seq)
    return agents

def main():
    ap=argparse.ArgumentParser()
    ap.add_argument('--map', required=True)
    ap.add_argument('--paths', required=True)
    ap.add_argument('--obstacles', default='@')  # 追加で壁を増やすならここに
    ap.add_argument('--swapxy', action='store_true')
    ap.add_argument('--allow-wait', dest='allow_wait', action='store_true', help='同マス滞在(待機)を許可')
    args=ap.parse_args()

    H,W,passable=load_movingai_map(args.map, args.obstacles)
    agents=load_eecbs_paths(args.paths)

    issues=0
    def ok4(a,b):
        dx=abs(a[0]-b[0]); dy=abs(a[1]-b[1])
        return (dx+dy)==1

    for ai,seq in enumerate(agents):
        prev=None
        for t,(x,y) in enumerate(seq):
            if args.swapxy:
                x,y = y,x
            # 範囲
            if not (0<=x<W and 0<=y<H):
                print(f"[BAD] Agent {ai} t={t} out-of-range at ({x},{y})")
                issues+=1; prev=(x,y); continue
            # 通行可能
            if not passable[y][x]:
                print(f"[BAD] Agent {ai} t={t} on obstacle at ({x},{y})")
                issues+=1
            # 4近傍 or 待機許可
            if prev is not None:
                if (x,y)!=prev:
                    if not ok4(prev,(x,y)):
                        print(f"[BAD] Agent {ai} step {t-1}->{t} not 4-neighbor: {prev}->{(x,y)}")
                        issues+=1
                else:
                    if not args.allow_wait:
                        print(f"[BAD] Agent {ai} step {t-1}->{t} stay on same cell without --allow-wait: {prev}")
                        issues+=1
            prev=(x,y)

    print("[OK] no issues" if issues==0 else f"[NG] found {issues} issues")

if __name__=="__main__":
    main()
