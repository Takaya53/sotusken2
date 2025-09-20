#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
to_visualizer_time.py
EECBS の出力 "Agent i: (r,c)->(r,c)->..." を
mapf-visualizer の time-major 形式
  t:(x0,y0),(x1,y1),...,(xN-1,yN-1)
に変換する。

使い方:
  python to_visualizer_time.py [MAP] [RAW] [OUT] [K]

  MAP: マップ (既定: ldk2.map)
  RAW: EECBS 出力 (既定: paths_raw.txt)
  OUT: 出力先     (既定: for_visualizer_time.txt)
    K: エージェント数の上限（昇順で先頭 K 台）。省略時は全エージェント

安全策:
- (row,col)->(x,y) 入替の自動判定（範囲外点の少ない方）
- エージェントIDの整列（昇順）
- 全エージェント系列を最大長に末尾パディング
- マップ範囲の簡易検証
"""

from __future__ import annotations
import re
import sys
import pathlib
from typing import Dict, List, Tuple, Optional

# -------------------------
# 引数処理
# -------------------------
args = sys.argv[1:]
MAP = pathlib.Path(args[0]) if len(args) >= 1 else pathlib.Path("ldk2.map")
SRC = pathlib.Path(args[1]) if len(args) >= 2 else pathlib.Path("paths_raw.txt")
DST = pathlib.Path(args[2]) if len(args) >= 3 else pathlib.Path("for_visualizer_time.txt")
K: Optional[int] = None
if len(args) >= 4:
    try:
        K = int(args[3])
        if K <= 0:
            K = None  # 非正なら無効扱い
    except ValueError:
        K = None

# -------------------------
# map header 読み取り（width/height）
# -------------------------
try:
    map_lines = MAP.read_text(encoding="utf-8").splitlines()
except FileNotFoundError:
    sys.exit(f"ERROR: マップファイルが見つかりません: {MAP}")

width = height = None
for ln in map_lines:
    s = ln.strip().lower()
    if s.startswith("width"):
        m = re.search(r"\d+", s)
        if m:
            width = int(m.group())
    elif s.startswith("height"):
        m = re.search(r"\d+", s)
        if m:
            height = int(m.group())
    if width is not None and height is not None:
        break

if width is None or height is None:
    sys.exit("ERROR: マップヘッダ（width/height）を取得できませんでした。")

# -------------------------
# EECBS 生出力のパース
#   行例: "Agent 12: (r,c)->(r,c)->..."
# -------------------------
try:
    raw_lines = SRC.read_text(encoding="utf-8").splitlines()
except FileNotFoundError:
    sys.exit(f"ERROR: 生出力ファイルが見つかりません: {SRC}")

agents_rc: Dict[int, List[Tuple[int, int]]] = {}  # (r,c) のまま一次保持
agent_pat = re.compile(r"\s*Agent\s*(\d+)\s*:\s*(.*)")
pair_pat  = re.compile(r"\(\s*(\d+)\s*,\s*(\d+)\s*\)")

for ln in raw_lines:
    m = agent_pat.match(ln)
    if not m:
        continue
    aid = int(m.group(1))
    rest = m.group(2)
    pts = pair_pat.findall(rest)
    if not pts:
        continue
    seq = [(int(r), int(c)) for (r, c) in pts]  # (r,c) として格納
    if seq:
        agents_rc.setdefault(aid, []).extend(seq)

if not agents_rc:
    sys.exit("ERROR: paths_raw.txt から経路を抽出できませんでした。")

# -------------------------
# エージェントIDの整列＆K台に制限（必要なら）
# -------------------------
ids_sorted = sorted(agents_rc.keys())
if K is not None:
    ids_sorted = ids_sorted[:min(K, len(ids_sorted))]

agents_rc = {aid: agents_rc[aid] for aid in ids_sorted}

# -------------------------
# (r,c) -> (x,y) 入替の自動判定
#   少数サンプルで範囲外点数を比較し、少ない方を採用
#   同数なら入替あり（swap=True）を優先
# -------------------------
def count_oob_from_pairs(pairs: List[Tuple[int, int]], swap: bool) -> int:
    oob = 0
    for r, c in pairs:
        x, y = (c, r) if swap else (r, c)
        if not (0 <= x < width and 0 <= y < height):
            oob += 1
    return oob

sample_pairs: List[Tuple[int, int]] = []
for aid in ids_sorted:
    sample_pairs.extend(agents_rc[aid][:64])  # 各エージェント先頭64点
    if len(sample_pairs) >= 1024:
        break

# サンプルが空になることは通常ないが、念のための防御
if not sample_pairs:
    USE_SWAP = True  # デフォルトは swap あり
else:
    oob_swap   = count_oob_from_pairs(sample_pairs, swap=True)
    oob_noswap = count_oob_from_pairs(sample_pairs, swap=False)
    USE_SWAP = (oob_swap <= oob_noswap)

# 変換本体 (r,c)->(x,y)
agents_xy: Dict[int, List[Tuple[int, int]]] = {}
for aid in ids_sorted:
    rc_path = agents_rc[aid]
    if USE_SWAP:
        xy_path = [(c, r) for (r, c) in rc_path]
    else:
        xy_path = [(r, c) for (r, c) in rc_path]
    agents_xy[aid] = xy_path

# -------------------------
# パス配列化＆最大長Tの決定・末尾パディング
# -------------------------
paths: List[List[Tuple[int, int]]] = [agents_xy[aid] for aid in ids_sorted if agents_xy[aid]]
if not paths:
    sys.exit("ERROR: 有効な経路がありません（全エージェントで点列が空）。")

N = len(paths)
T = max(len(p) for p in paths)

for p in paths:
    if len(p) < T:
        p.extend([p[-1]] * (T - len(p)))
    elif len(p) > T:
        del p[T:]

# -------------------------
# 範囲チェック（先頭の異常のみ警告）
# -------------------------
warn = None
for i, p in enumerate(paths):
    for t, (x, y) in enumerate(p):
        if not (0 <= x < width and 0 <= y < height):
            warn = (i, t, x, y)
            break
    if warn:
        break

head_msg = f"OK: map=({width}x{height}) N={N} T={T} swap={'on' if USE_SWAP else 'off'}"
if K is not None:
    head_msg += f" (K={K})"
if warn:
    print(f"WARNING: 範囲外座標を検出 agent={warn[0]} t={warn[1]} pos=({warn[2]},{warn[3]}) | {head_msg}")
else:
    print(head_msg)

# -------------------------
# time-major 書き出し
#   各行: t:(x0,y0),(x1,y1),...,(xN-1,yN-1)
# -------------------------
try:
    if str(DST.parent) not in ("", "."):
        DST.parent.mkdir(parents=True, exist_ok=True)
    with DST.open("w", encoding="utf-8", newline="\n") as g:
        for t in range(T):
            row = ",".join(f"({paths[a][t][0]},{paths[a][t][1]})" for a in range(N))
            g.write(f"{t}:{row},\n")    # ← コロンの直後にスペースを入れない
except OSError as e:
    sys.exit(f"ERROR: 出力に失敗しました: {DST} ({e})")

print(f"[OK] wrote {DST}  N={N}  T={T}")