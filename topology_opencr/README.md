# OpenCR Base Topology Optimization

PythonでOpenCR 1.0用の土台形状をトポロジー最適化する試作ツールです。SIMP法、感度フィルタ、OC法を使い、最適化密度、二値形状、SVG、押し出しSTLを出力します。

## モデルの前提

- OpenCR 1.0の公式外形寸法は `105 x 75 mm`、重量は `60 g` です。
- 初期設定の設計領域は、基板外周に10 mmずつ余白を加えた `125 x 95 mm` です。
- OpenCR取付穴は公式穴位置を確定できていないため、基板四隅から5 mm内側を仮定しています。必ず実物または公式CADで測定して変更してください。
- この解析は縦置きブラケットの面内荷重を想定した2D平面応力モデルです。水平な棚板の面外曲げ、ねじ締結、振動、衝撃、3Dプリントの異方性は評価しません。
- STLは二値化した1 mm角セルを押し出した形状です。CADで外周を平滑化し、穴径、公差、フィレットを調整してから製作してください。

## セットアップ

```bash
cd /Users/yuheitakeda/Documents/robot_arm
python3 -m venv .venv
source .venv/bin/activate
pip install -r topology_opencr/requirements.txt
```

## 実行

```bash
python topology_opencr/opencr_base_topopt.py
```

出力先は `topology_opencr/results/` です。

- `density.png`: 最適化された材料密度
- `convergence.png`: コンプライアンスの収束履歴
- `density.npy`, `mask.npy`: 数値データ
- `opencr_base.svg`: 2D形状
- `opencr_base.stl`: 指定厚さで押し出した3D形状

## 穴位置を変更する

座標原点は設計領域の左下、単位はmmです。OpenCR穴と筐体側固定穴を実測値に変更してください。

```bash
python topology_opencr/opencr_base_topopt.py \
  --board-holes "15,15;110,15;15,80;110,80" \
  --frame-holes "5,5;120,5;5,90;120,90"
```

OpenCR基板の左下を設計領域の `(10, 10)` に置く場合、基板内座標 `(x, y)` は設計領域座標 `(x + 10, y + 10)` に変換します。

## 主な調整項目

```bash
python topology_opencr/opencr_base_topopt.py \
  --volfrac 0.35 \
  --rmin 3.0 \
  --thickness 4.0 \
  --hole-radius 1.7 \
  --pad-radius 6.0 \
  --load-x 0 \
  --load-y -1
```

- `--volfrac`: 材料使用率。小さいほど軽量ですが細くなります。
- `--rmin`: 最小部材寸法に影響するフィルタ半径です。
- `--hole-radius`: 取付穴半径です。プリント公差を含めて設定します。
- `--pad-radius`: 穴周辺で必ず残す材料の半径です。
- `--load-x`, `--load-y`: 面内荷重方向です。

## 製作前の確認

1. OpenCRの取付穴位置とコネクタ干渉領域を実測する。
2. 使用するねじ、スペーサ、ワッシャに合わせて穴径とパッド径を決める。
3. 実際の固定方法と最大荷重を使って3D FEAを行う。
4. 積層方向、フィレット、最小肉厚をCADで調整する。
5. 低荷重で試作し、振動と衝撃を含めて検証する。

## 参考

- [ROBOTIS OpenCR 1.0 e-Manual](https://emanual.robotis.com/docs/en/parts/controller/opencr10_jp/)
- [ROBOTIS OpenCR Hardware](https://github.com/ROBOTIS-GIT/OpenCR-Hardware)
