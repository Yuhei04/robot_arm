# UbuntuでCAD座標が壊れる場合の復旧手順

Mac側では直っているのにUbuntu側でモータや部品の座標が壊れる場合、Ubuntu側に古いCSVや古い生成スクリプトが残っている可能性が高い。

この手順では、Mac側の最新版をUbuntuへ同期し、URDFを再生成する。

## 1. Ubuntu側で作業ディレクトリへ移動

```sh
cd ~/robot_arm
source .venv-ubuntu/bin/activate
```

## 2. Mac側から最新版をコピーする

Mac側で実行する。

```sh
rsync -av \
  /Users/yuheitakeda/Documents/robot_arm/tools/ \
  USER@UBUNTU_HOST:~/robot_arm/tools/

rsync -av \
  /Users/yuheitakeda/Documents/robot_arm/cad/exports/fusion_tables/ \
  USER@UBUNTU_HOST:~/robot_arm/cad/exports/fusion_tables/

rsync -av \
  /Users/yuheitakeda/Documents/robot_arm/cad/exports/fusion_meshes/ \
  USER@UBUNTU_HOST:~/robot_arm/cad/exports/fusion_meshes/
```

`USER` と `UBUNTU_HOST` はUbuntu側のユーザー名とホスト名/IPに置き換える。

最低限必要なのは次。

```text
tools/build_fusion_visual_urdf.py
tools/build_fusion_jointed_urdf.py
tools/fusion360_export_urdf_data/
cad/exports/fusion_tables/occurrences.csv
cad/exports/fusion_tables/link_assignment_template.csv
cad/exports/fusion_tables/joints_template.csv
cad/exports/fusion_meshes/
```

## 3. Ubuntu側で固定表示URDFを再生成する

Ubuntu側で実行する。

```sh
cd ~/robot_arm
source .venv-ubuntu/bin/activate

make fusion-visual
python tools/view_urdf_pybullet.py --urdf urdf/robot_arm_fusion_visual.urdf
```

この時点で、Fusion上の形に近く表示されるか確認する。

## 4. CSVが新しいか確認する

Ubuntu側では、まず次でまとめて確認できる。

```sh
make check-coordinates
```

手で詳しく見る場合は次を実行する。

```sh
cd ~/robot_arm
source .venv-ubuntu/bin/activate

python - <<'PY'
import csv
from pathlib import Path

rows = list(csv.DictReader(Path("cad/exports/fusion_tables/occurrences.csv").open()))

targets = [
    "XC-430_idle_1_M_DC11_A01_IDLER_ASM_1_M_DC11_A01_IDLER_1.stl",
    "XC-430_idle_1_M_DC11_A01_HORN_ASM_1_M_DC11_A01_HORN_1.stl",
    "2XC-430_1_DC11_A01_HORN_REF_ASM_1_DC11_A01_HORN_REF_1.stl",
]

for mesh in targets:
    for row in rows:
        if row["mesh_file"] == mesh:
            print(mesh)
            print("  origin_cm:", row["origin_x_cm"], row["origin_y_cm"], row["origin_z_cm"])
            print("  has_matrix:", bool(row.get("m00")))
            break
    else:
        print(mesh)
        print("  not found")
PY
```

正常なら、次のようになる。

- `has_matrix: True` が出る
- モータ内部部品の `origin_cm` が `0,0,0` 付近ではなく、アーム上の座標になっている

## 5. 結果ごとの判断

### `has_matrix: False` の場合

Ubuntu側の `occurrences.csv` が古い。

対処:

1. Mac側でFusionスクリプトを再実行する
2. `cad/exports/fusion_tables/occurrences.csv` をUbuntuへコピーし直す
3. Ubuntu側で `python tools/build_fusion_visual_urdf.py` を再実行する

### `origin_cm` が `0,0,0` 付近の場合

Mac側のFusionエクスポートスクリプトが古い可能性が高い。

対処:

1. Mac側のFusion Scriptsフォルダに最新版の `fusion360_export_urdf_data.py` を入れる
2. Fusion 360で `fusion360_export_urdf_data` を再実行する
3. `cad/exports/fusion_tables/occurrences.csv` をUbuntuへコピーし直す
4. Ubuntu側で固定表示URDFを再生成する

### 親モータSTLだけが変な位置に出る場合

現在の `tools/build_fusion_visual_urdf.py` は、子部品が存在する親STLを除外する前提。

確認:

```sh
grep -E "XC-430_idle_[12]\\.stl|2XC-430_1\\.stl|XM_H-430_idler_[12]\\.stl" urdf/robot_arm_fusion_visual.urdf
```

何も出なければ、親モータSTLは除外されている。

もし出る場合は、Ubuntu側の `tools/build_fusion_visual_urdf.py` が古い。

## 6. 再確認用コマンド

固定表示:

```sh
make fusion-visual
python tools/view_urdf_pybullet.py --urdf urdf/robot_arm_fusion_visual.urdf
```

PNG出力:

```sh
python tools/render_urdf_pybullet.py \
  --urdf urdf/robot_arm_fusion_visual.urdf \
  --output outputs/robot_arm_fusion_visual_ubuntu_fixed.png

# 可動URDFをPNGで確認する場合
make render-fusion-urdf
```

## 7. 固定表示が直った後

固定表示が正しくなってから、可動URDFを再生成する。

```sh
make fusion-jointed
make view-fusion-urdf
```

可動URDFで形が崩れる場合は、次を調整する。

```text
cad/exports/fusion_tables/link_assignment_template.csv
cad/exports/fusion_tables/joints_template.csv
```

固定表示が壊れている状態で可動URDFを調整してはいけない。まず固定表示を正しくする。
