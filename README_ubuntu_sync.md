# Ubuntu反映手順

このフォルダは、Mac側で修正したロボットアームのCADメッシュ、設定CSV、URDF、PyBullet確認用スクリプトをUbuntuへ反映するためのパッケージです。

## 1. 中身

```text
ubuntu_sync_package/
  Makefile
  cad/exports/fusion_meshes/        STLメッシュ
  cad/exports/fusion_tables/        表示設定、リンク割り当て、ジョイント設定CSV
  tools/                            URDF生成、PyBullet表示、画像出力スクリプト
  urdf/                             生成済みURDF
  outputs/                          確認画像の出力先
```

重要な編集ファイル:

```text
cad/exports/fusion_tables/visual_link_config.csv
cad/exports/fusion_tables/joints_template.csv
```

`visual_link_config.csv` は、どのSTLを表示するか、どのリンクに属するかを決める表です。

```text
include=1   表示する
include=0   表示しない
link_name   linkA から linkG のどれに属するか
```

リンク名:

```text
linkA: base
linkB: base yaw
linkC: shoulder
linkD: upper arm
linkE: forearm
linkF: elbow roll
linkG: wrist
```

`joints_template.csv` はジョイントの親子関係、回転軸、回転中心を決める表です。

```text
origin_x_mm, origin_y_mm, origin_z_mm
```

は、親リンク座標から見たジョイント中心です。

## 2. MacからUbuntuへ転送

Mac側で実行:

```sh
cd /Users/yuheitakeda/Documents/robot_arm

rsync -av ubuntu_sync_package/ yuhei@yuhei-ubuntu:~/robot_arm/
```

Ubuntu側に `~/robot_arm` が無い場合は先に作ります。

```sh
ssh yuhei@yuhei-ubuntu 'mkdir -p ~/robot_arm'
rsync -av ubuntu_sync_package/ yuhei@yuhei-ubuntu:~/robot_arm/
```

## 3. Ubuntu側の準備

Ubuntu側で実行:

```sh
cd ~/robot_arm

python3 -m venv .venv-pybullet
source .venv-pybullet/bin/activate
pip install pybullet
```

このパッケージの `Makefile` は標準で `.conda-pybullet/bin/python` を見ます。Ubuntuではvenvを使うなら、コマンド実行時に `PYBULLET_PY` を指定します。

```sh
export PYBULLET_PY=.venv-pybullet/bin/python
```

毎回指定する場合:

```sh
make fusion-jointed PYBULLET_PY=.venv-pybullet/bin/python
```

## 4. URDFを再生成

Ubuntu側で実行:

```sh
cd ~/robot_arm
export PYBULLET_PY=.venv-pybullet/bin/python

python3 tools/create_link_assignment_template.py
make fusion-visual
make fusion-jointed
```

注意:

```sh
make fusion-config
```

は `visual_link_config.csv` をCAD exportから作り直します。Mac側で手修正した `include` や `link_name` を維持したい場合は実行しないでください。

## 5. PyBulletで確認

GUIで確認:

```sh
cd ~/robot_arm
export PYBULLET_PY=.venv-pybullet/bin/python

make view-fusion
```

画像で確認:

```sh
cd ~/robot_arm
export PYBULLET_PY=.venv-pybullet/bin/python

make render-fusion
```

画像はここに出ます。

```text
outputs/robot_arm_fusion_rebuilt.png
```

固定表示だけ確認する場合:

```sh
$PYBULLET_PY tools/render_urdf_pybullet.py \
  --urdf urdf/robot_arm_fusion_visual.urdf \
  --output outputs/check_visual.png
```

## 6. 修正後の反映手順

リンク割り振りを直した場合:

```sh
vim cad/exports/fusion_tables/visual_link_config.csv

python3 tools/create_link_assignment_template.py
make fusion-jointed
make view-fusion
```

ジョイント中心や軸を直した場合:

```sh
vim cad/exports/fusion_tables/joints_template.csv

make fusion-jointed
make view-fusion
```

## 7. 現在のジョイント中心

現在の `joints_template.csv` では、ワールド座標で下から順に次の高さに回転中心が来るように設定しています。

```text
base_yaw_joint        z=22.0 mm
shoulder_pitch_joint  z=95.5 mm
upper_arm_roll_joint  z=202.2 mm
elbow_pitch_joint     z=254.2 mm
elbow_roll_joint      z=323.2 mm
wrist_pitch_joint     z=390.7 mm
```

## 8. よくある注意点

`fr12_h101_k_2.stl` はSTLファイルとして存在しますが、現在の `occurrences.csv` に配置情報が無いため、`visual_link_config.csv` では `orphan_mesh` として `include=0` になっています。

表示したい場合は、Fusion側で該当部品を表示ONにして再エクスポートし、`occurrences.csv` に配置行が出る状態にするのが正しいです。配置行列なしでURDFへ入れると、見た目は合っても座標が信用できません。
