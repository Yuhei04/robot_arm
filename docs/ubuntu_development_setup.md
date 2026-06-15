# Ubuntuでロボットアーム開発環境を作る手順

このメモは、Macで進めていたロボットアーム開発をUbuntu側へ移し、ROS2やGPUを使える開発環境にするための手順。

Mac/Fusion 360で出力したCADデータをUbuntu側へ移して、まずPyBulletでURDF確認と調整を行う。その後、同じURDFをROS2ワークスペースへ取り込み、制御・可視化・シミュレーションへ進める。

## Ubuntuへ移す理由

- ROS2の公式対応がUbuntu中心で、開発・実機連携・デバッグがやりやすい
- NVIDIA GPUを使うシミュレーション、画像処理、学習、推論を同じマシンで扱いやすい
- 実機制御用のUSBデバイス、シリアル、udev、systemdなどをMacより直接扱いやすい
- Mac側のFusion 360作業と、Ubuntu側のロボット開発作業を分離できる

## 推奨する前提

Ubuntuは、ROS2のLTSに合わせる。

```text
Ubuntu 24.04 LTS
ROS2 Jazzy Jalisco
```

ROS2 JazzyはUbuntu 24.04向けのLTS。Ubuntu 22.04を使う場合はROS2 Humbleになるため、以降のROS2コマンドやパッケージ名を読み替える。

GPUを使う場合は、先にUbuntu上でNVIDIAドライバが正常に動く状態にしておく。CUDA Toolkitは、深層学習やCUDAコードをビルドする必要が出てから入れればよい。PyBulletやROS2の基本確認だけなら、最初はNVIDIAドライバと `nvidia-smi` の確認で十分。

## 役割分担

- Mac: Fusion 360でアセンブリ編集、STL/CSVのエクスポート
- Ubuntu: PyBulletで表示確認、リンク割り当て、可動URDFの調整
- Ubuntu: ROS2ワークスペース、RViz、実機制御、GPUを使う処理

Fusion 360が必要な作業はMac側で行う。Ubuntu側では、すでに出力された `cad/exports/` を使ってURDFを生成・確認する。

## Ubuntuへ移すフォルダ

基本はプロジェクト全体を移す。

```text
/Users/yuheitakeda/Documents/robot_arm
```

最低限必要なものは次。

```text
cad/exports/fusion_meshes/
cad/exports/fusion_tables/
tools/
urdf/
docs/
Makefile
```

Macで作った仮想環境はUbuntuでは使わないので、移さなくてもよい。

```text
.conda-pybullet/
.venv/
.venv-pybullet/
.DS_Store
```

## コピー方法の例

外付けSSDやUSBでコピーしてもよい。ネットワークで送るなら、Mac側から次のように送れる。

```sh
rsync -av \
  --exclude .conda-pybullet \
  --exclude .venv \
  --exclude .venv-pybullet \
  --exclude .DS_Store \
  /Users/yuheitakeda/Documents/robot_arm/ \
  USER@UBUNTU_HOST:~/robot_arm/
```

`USER` と `UBUNTU_HOST` はUbuntu側のユーザー名とホスト名/IPに置き換える。

## Ubuntu側の初期セットアップ

```sh
sudo apt update
sudo apt install -y \
  build-essential \
  curl \
  git \
  python3 \
  python3-pip \
  python3-venv \
  rsync
```

PyBullet用の仮想環境を作る。

```sh
cd ~/robot_arm
python3 -m venv .venv-ubuntu
source .venv-ubuntu/bin/activate
pip install --upgrade pip
pip install pybullet
```

Makefileを使う場合は次でもよい。

```sh
make setup-pybullet
```

確認:

```sh
python -c "import pybullet as p; print(p.__version__ if hasattr(p, '__version__') else 'pybullet ok')"
```

## GPUを確認する

NVIDIA GPUを使う場合は、まずUbuntuの「追加のドライバー」またはUbuntu標準のドライバ管理で推奨ドライバを入れる。CUDA Toolkitを最初から入れるより、ドライバ単体が安定していることを先に確認する。

確認:

```sh
nvidia-smi
```

ここでGPU名、ドライババージョン、CUDA Versionが表示されれば、ドライバは動いている。

CUDA Toolkitが必要になる例:

- CUDAコードを自分でビルドする
- GPU版のPyTorch、ONNX Runtime、Isaac Simなどを使う
- 画像処理や学習をGPUで回す

不要な例:

- URDFを生成する
- PyBulletで形状確認する
- ROS2の基本ノードを動かす
- RVizでURDFを表示する

## ROS2を入れる

Ubuntu 24.04ならROS2 Jazzyを入れる。基本は公式のDebianパッケージを使う。

公式手順に従ってROS2のaptリポジトリを追加した後、次を入れる。

```sh
sudo apt update
sudo apt install -y \
  ros-jazzy-desktop \
  python3-colcon-common-extensions \
  python3-rosdep
```

初回だけrosdepを初期化する。すでに初期化済みならエラーになってもよい。

```sh
sudo rosdep init
rosdep update
```

インストール後、毎回ROS2環境を読み込む。

```sh
source /opt/ros/jazzy/setup.bash
```

毎回打つのが面倒なら、Ubuntu側のシェル設定に追加する。

```sh
echo "source /opt/ros/jazzy/setup.bash" >> ~/.bashrc
```

確認:

```sh
ros2 --version
ros2 run demo_nodes_cpp talker
```

別ターミナルで:

```sh
source /opt/ros/jazzy/setup.bash
ros2 run demo_nodes_py listener
```

`talker` と `listener` が通信できればROS2の基本セットアップは通っている。

## ROS2ワークスペースを作る

このリポジトリは、最初はCAD/URDF生成用としてそのまま使う。ROS2パッケージは別のワークスペースに置くと整理しやすい。

```sh
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws
source /opt/ros/jazzy/setup.bash
colcon build
source install/setup.bash
```

シェル設定に追加する場合:

```sh
echo "source ~/ros2_ws/install/setup.bash" >> ~/.bashrc
```

最初の構成案:

```text
~/robot_arm/
  cad/
  tools/
  urdf/
  docs/

~/ros2_ws/src/
  robot_arm_description/
  robot_arm_bringup/
  robot_arm_control/
```

役割:

```text
robot_arm_description: URDF、mesh、RViz設定
robot_arm_bringup: 起動launch、実機/シミュレーション切り替え
robot_arm_control: DYNAMIXELやOpenCRなどの制御ノード
```

まずは `~/robot_arm/urdf/robot_arm_fusion.urdf` と `cad/exports/fusion_meshes/` を `robot_arm_description` に取り込む。PyBulletで形状と関節が固まってからROS2側へ移すと、ROS2パッケージ側の修正が少なくなる。



## 固定表示URDFを確認する

まずは関節なしの固定表示で、Fusionの形状に近いか確認する。

```sh
cd ~/robot_arm
source .venv-ubuntu/bin/activate
make fusion-visual
python tools/view_urdf_pybullet.py --urdf urdf/robot_arm_fusion_visual.urdf
```

現在の固定表示URDFでは、Fusionの親アセンブリSTLが間違った位置に出るケースを避けるため、子部品が存在する親STLは除外している。

確認すること:

- アーム全体の形がFusionと大きく違っていないか
- 余計なモータが二重表示されていないか
- モータだけ変な位置に飛んでいないか
- スケールが極端に大きい/小さい状態になっていないか

PyBulletの `No inertial data for link...` は表示確認では無視してよい。

## PNGで軽く確認する

GUIが重い場合は、先に画像だけ出す。

```sh
python tools/render_urdf_pybullet.py \
  --urdf urdf/robot_arm_fusion_visual.urdf \
  --output outputs/robot_arm_fusion_visual_ubuntu.png

# 可動URDFをPNGで確認する場合
make render-fusion-urdf
```

出力:

```text
outputs/robot_arm_fusion_visual_ubuntu.png
```

## 可動URDFを生成する

固定表示が正しくなってから行う。

```sh
make fusion-jointed
make view-fusion-urdf
```

この時点の可動URDFは、まだ調整用の仮モデル。リンク割り当てと関節原点を詰める必要がある。

## 調整するCSV

### リンク割り当て

```text
cad/exports/fusion_tables/link_assignment_template.csv
```

`link_name` に、各STLが属するURDFリンク名を入れる。

例:

```text
base_link
base_yaw_link
shoulder_pitch_link
upper_arm_link
forearm_link
elbow_roll_link
wrist_pitch_link
```

判断基準:

- ある関節で一体に動く部品は同じlink
- 関節をまたいで相対回転する部品は別link
- 親モータSTLがずれる場合は、親ではなく子部品STLを使う

### 関節情報

```text
cad/exports/fusion_tables/joints_template.csv
```

各関節の軸と原点を調整する。

```csv
joint_name,parent_link,child_link,motor_id,axis_x,axis_y,axis_z,origin_x_mm,origin_y_mm,origin_z_mm,sign
```

重要:

- `axis_x/y/z` は親リンク座標系で見た回転軸
- `origin_x/y/z_mm` は親リンクから見た関節原点
- `sign` はDYNAMIXELの角度符号との対応

## 軸の確認順

PyBulletのスライダーで、1軸ずつ動かす。

```text
j1_base_yaw
j2_shoulder_pitch
j6_upper_arm_roll
j3_elbow_pitch
j4_elbow_roll
j5_wrist_pitch
```

確認すること:

- 回るべき部品だけが回るか
- 回転中心がモータ軸に合っているか
- 正方向が実機の観察結果と合っているか

実機で確認済みの符号候補:

| ID | 関節 | 符号 |
| --- | --- | --- |
| 1 | ベース旋回 | `+1` |
| 2 | 肩ピッチ | `+1` |
| 3 | 肘ピッチ | `-1` |
| 4 | 肘ロール | `-1` |
| 5 | 手首ピッチ | `+1` |
| 6 | 上腕ロール | `-1` |

## MacでFusionを更新した後

Mac側でFusionから再エクスポートしたら、Ubuntuへ次を再コピーする。

```text
cad/exports/fusion_meshes/
cad/exports/fusion_tables/occurrences.csv
```

その後、Ubuntu側で再生成する。

```sh
cd ~/robot_arm
source .venv-ubuntu/bin/activate
python tools/build_fusion_visual_urdf.py
python tools/view_urdf_pybullet.py --urdf urdf/robot_arm_fusion_visual.urdf
```

リンク割り当てCSVを再生成すると手作業の修正が消える可能性があるので、`link_assignment_template.csv` は必要なときだけ更新する。

## よくある問題

### GUIが開かない

Ubuntuのデスクトップ環境上で実行する。SSH越しの場合はX11転送やVNCが必要。

### `No inertial data...` が出る

表示確認では無視してよい。動力学シミュレーションをする段階で、質量・慣性を追加する。

### モータが二重に表示される

親アセンブリSTLと子部品STLが両方表示されている可能性が高い。`tools/build_fusion_visual_urdf.py` の除外ルール、または `occurrences.csv` の対象メッシュを確認する。

### モータだけ位置がずれる

Fusionのネストされたoccurrenceのtransformが合っていない可能性がある。エクスポートスクリプトは `transform2` を使う前提なので、Mac側のFusionスクリプトが最新版か確認する。

### スケールがおかしい

STLはmm、URDFはmとして扱っている。URDF内のmesh scaleは次が基本。

```xml
scale="0.001 0.001 0.001"
```

## まず目指す状態

1. `robot_arm_fusion_visual.urdf` がFusionと同じ形で見える
2. 余計なモータ重複がない
3. `robot_arm_fusion.urdf` で各スライダーを1軸ずつ動かせる
4. 回る部品と回転中心をCSVで調整できる

ここまでできたら、次は逆運動学に使うための正確なリンク長、関節軸、ツール先端位置を確定する。

## 参考

- ROS2 Jazzy Installation: https://docs.ros.org/en/jazzy/Installation.html
- ROS2 Jazzy Ubuntu deb packages: https://docs.ros.org/en/jazzy/Installation/Ubuntu-Install-Debs.html
- NVIDIA Ubuntu Driver Installation Guide: https://docs.nvidia.com/datacenter/tesla/driver-installation-guide/latest/ubuntu.html
- CUDA Installation Guide for Linux: https://docs.nvidia.com/cuda/cuda-installation-guide-linux/index.html
