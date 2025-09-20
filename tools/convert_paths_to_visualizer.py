#!/usr/bin/env python3
import argparse, re, sys

# ---- MovingAIマップ読込（障害物文字は既定'@'、追加可） ----
def load_movingai_map(path, obstacles='@'):
    H = W = None
    grid = []
    with open(path, 'r', encoding='utf-8', errors='ignore') as f:
        lines = [ln.rstrip('\n') for ln in f]
    for i, ln in enumerate(lines):
        ls = ln.strip().lower()
        if ls.startswith('height'):
            H = int(ln.split()[-1])
        if ls.startswith('width'):
            W = int(ln.split()[-1])
        if ls == 'map':
            if H is None or W is None:
                print("[ERR] header has no height/width", file=sys.stderr)
                sys.exit(2)
            grid = lines[i+1:i+1+H]
            break
    if len(grid) != H or any(len(r) != W for r in grid):
        print(f"[ERR] grid size mismatch H={H} W={W} real=({len(grid)} x {len(grid[0]) if grid else 0})", file=sys.stderr)
        sys.exit(2)
    obs = set(obstacles)
    passable = [[(c not in obs) for c in row] for row in grid]
    return H, W, passable

# ---- EECBSのpaths形式を読込 ----
pat_path_line = re.compile(r'^Agent\s+(\d+):\s*(.*)$')
pat_point = re.compile(r'\((\d+),\s*(\d+)\)')

def load_eecbs_paths(path):
    agents = []
    with open(path, 'r', encoding='utf-8', errors='ignore') as f:
        for ln in f:
            m = pat_path_line.match(ln.strip())
            if not m:
                continue
            seq = [(int(x), int(y)) for x, y in pat_point.findall(m.group(2))]
            agents.append(seq)
    return agents

def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--map', required=True, help='MovingAI形式マップ')
    ap.add_argument('--paths', required=True, help='EECBSのpaths_*.txt')
    ap.add_argument('--out', required=True, help='visualizer用の出力ファイル')
    ap.add_argument('--obstacles', default='@', help="障害物として扱う文字の集合（例 '@#T'）")
    ap.add_argument('--swapxy', action='store_true', help='(x,y)→(y,x)に入替えて出力')
    ap.add_argument('--allow-wait', dest='allow_wait', action='store_true', help='同一セル滞在を許可（検査用）')
    ap.add_argument('--repeat', type=int, default=1, help='各時刻をこの回数だけ複製（再生をゆっくりに）')
    ap.add_argument('--pad', type=int, default=0, help='末尾にゴール滞在フレームを追加')
    ap.add_argument('--only', type=int, default=0, help='先頭N体だけ出力（0は全員）')
    ap.add_argument('--id-offset', type=int, default=0, help='出力IDのオフセット（既定0）')
    ap.add_argument('--strict', action='store_true', help='検査で問題があれば終了')
    args = ap.parse_args()

    H, W, passable = load_movingai_map(args.map, args.obstacles)
    agents = load_eecbs_paths(args.paths)

    if args.only > 0:
        agents = agents[:args.only]

    def in_range(x, y): return 0 <= x < W and 0 <= y < H
    def ok4(a, b): 
        dx = abs(a[0] - b[0]); dy = abs(a[1] - b[1])
        return (dx + dy) == 1

    # --- 軽い検査（警告はstderrへ）。strictの場合は問題数>0で終了 ---
    issues = 0
    for ai, seq in enumerate(agents):
        prev = None
        for t, (x, y) in enumerate(seq):
            if args.swapxy:
                x, y = y, x
            if not in_range(x, y):
                print(f"[BAD] Agent {ai} t={t} out-of-range ({x},{y})", file=sys.stderr)
                issues += 1
            elif not passable[y][x]:
                print(f"[BAD] Agent {ai} t={t} on obstacle ({x},{y})", file=sys.stderr)
                issues += 1
            if prev is not None:
                if (x, y) != prev and not ok4(prev, (x, y)):
                    print(f"[BAD] Agent {ai} step {t-1}->{t} not 4-neighbor: {prev}->{(x,y)}", file=sys.stderr)
                    issues += 1
                if (x, y) == prev and not args.allow_wait:
                    print(f"[BAD] Agent {ai} stay without --allow-wait at t={t}: {prev}", file=sys.stderr)
                    issues += 1
            prev = (x, y)
    if issues > 0:
        msg = f"[WARN] found {issues} issues"
        if args.strict:
            print(msg, file=sys.stderr)
            sys.exit(3)
        else:
            print(msg + " (continue)", file=sys.stderr)
    else:
        print("[OK] validation passed", file=sys.stderr)

    # --- 生成（repeat/pad/IDオフセット適用） ---
    with open(args.out, 'w', encoding='utf-8') as g:
        for out_id, seq in enumerate(agents, start=args.id_offset):
            # swapは出力時点でも適用（上の検査と同じ向きで書き出し）
            seq2 = [(y, x) if args.swapxy else (x, y) for (x, y) in seq]
            ext = []
            for p in seq2:
                ext.extend([p] * max(1, args.repeat))
            if args.pad > 0 and ext:
                ext.extend([ext[-1]] * args.pad)
            line = str(out_id) + ":" + ",".join(f"({x},{y})" for (x, y) in ext)
            g.write(line + "\n")

if __name__ == "__main__":
    main()