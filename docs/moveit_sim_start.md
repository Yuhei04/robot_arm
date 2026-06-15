# MoveItでシミュレータ上のロボットアームを動かす最初の手順

最初はGazeboなどの物理シミュレータではなく、Fusionから作った実モデルURDFをMoveIt + RViz + fake controllerで動かす。

この段階で確認すること:

- URDFをMoveItに読み込める
- Planning Groupを作れる
- RViz上で目標姿勢を指定できる
- `Plan & Execute` で関節が動く

物理演算、接触、重力、モータモデルは次の段階で扱う。まずはMoveItの軌道計画が通る状態を作る。

## 前提

Ubuntu 24.04 + ROS 2 Jazzyを想定する。

```sh
source /opt/ros/jazzy/setup.bash
```

MoveItがまだ入っていない場合:

```sh
sudo apt update
sudo apt install -y \
  ros-jazzy-moveit \
  ros-jazzy-moveit-setup-assistant \
  ros-jazzy-ros2-control \
  ros-jazzy-ros2-controllers \
  ros-jazzy-joint-state-publisher-gui \
  ros-jazzy-xacro \
  python3-colcon-common-extensions
```

確認:

```sh
ros2 launch moveit_setup_assistant setup_assistant.launch.py
```

MoveIt Setup Assistantが開けばOK。

## 1. まず使うURDF

Fusion/CADから作った実モデルを使う。

```text
urdf/robot_arm_fusion.urdf
```

このURDFは `cad/exports/fusion_meshes` のSTLを参照するため、MoveIt用パッケージにはURDFだけでなくメッシュも入れる。

使う関節:

```text
base_yaw_joint
shoulder_pitch_joint
upper_arm_roll_joint
elbow_pitch_joint
elbow_roll_joint
wrist_pitch_joint
```

手先リンク:

```text
linkG
```

このURDFはvisual中心なので、最初のMoveIt確認には十分。Gazeboなどで物理シミュレーションするには、後でcollision、inertial、ros2_control設定を足す。

## 2. ROS 2ワークスペースを作る

```sh
mkdir -p ~/arm_ws/src
cd ~/arm_ws/src

ros2 pkg create robot_arm_description --build-type ament_cmake
mkdir -p robot_arm_description/urdf
if [ -f ~/robot_arm/urdf/robot_arm_fusion.urdf ]; then
  ROBOT_ARM_DIR=~/robot_arm
elif [ -f ~/Documents/robot_arm/robot_arm/urdf/robot_arm_fusion.urdf ]; then
  ROBOT_ARM_DIR=~/Documents/robot_arm/robot_arm
else
  echo "robot_arm_fusion.urdf が見つかりません"
  echo "次で場所を探してください:"
  echo "find ~ -path '*/urdf/robot_arm_fusion.urdf' -print"
  exit 1
fi

mkdir -p robot_arm_description/cad/exports
cp -a "$ROBOT_ARM_DIR/cad/exports/fusion_meshes" \
  robot_arm_description/cad/exports/

sed 's#filename="../cad/exports/fusion_meshes/#filename="package://robot_arm_description/cad/exports/fusion_meshes/#g' \
  "$ROBOT_ARM_DIR/urdf/robot_arm_fusion.urdf" \
  > robot_arm_description/urdf/robot_arm_fusion.urdf
```

`robot_arm_description/CMakeLists.txt` にインストール設定を追加する。

```cmake
install(
  DIRECTORY urdf cad
  DESTINATION share/${PROJECT_NAME}
)
```

ビルド確認:

```sh
cd ~/arm_ws
source /opt/ros/jazzy/setup.bash
colcon build --packages-select robot_arm_description
source install/setup.bash
```

## 3. MoveIt設定パッケージを生成する

MoveIt Setup Assistantを起動する。

```sh
cd ~/arm_ws
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 launch moveit_setup_assistant setup_assistant.launch.py
```

Setup Assistantで行うこと:

```text
1. Create New MoveIt Configuration Package
2. URDF:
   ~/arm_ws/src/robot_arm_description/urdf/robot_arm_fusion.urdf
3. Self-Collisions:
   Generate Collision Matrix
4. Virtual Joints:
   child link: linkA
   parent frame: world
   type: fixed
5. Planning Groups:
   group name: arm
   kinematic chain:
     linkA -> linkG
6. Robot Poses:
   最初は未設定でよい
   Add Poseで落ちる場合はスキップして進む
   home姿勢は後でSRDFに追加できる
7. End Effectors:
   最初は未設定でよい
8. ROS 2 Control / Controllers:
   最初は fake controller を使う
9. Author Information:
   名前とメールを入力
10. Configuration Files:
   package path:
   ~/arm_ws/src/robot_arm_moveit_config
   Generate Package
```

生成後にビルドする。

```sh
cd ~/arm_ws
source /opt/ros/jazzy/setup.bash
colcon build
source install/setup.bash
```

## 4. RViz上で動かす

```sh
ros2 launch robot_arm_moveit_config demo.launch.py
```

RVizが開いたら、MotionPlanningパネルで次を試す。

```text
Planning Group: arm
Start State: <current>
Goal State: random valid
Plan
Execute
```

`Plan` が成功し、`Execute` でRViz上のロボット姿勢が動けば、MoveItの最初のシミュレーション確認は成功。

## 5. 動かないときの確認

MoveIt Setup Assistantが起動しない:

```sh
sudo apt install ros-jazzy-moveit-setup-assistant
source /opt/ros/jazzy/setup.bash
```

`demo.launch.py` が見つからない:

```sh
cd ~/arm_ws
colcon build
source install/setup.bash
ros2 pkg list | grep robot_arm
```

Planning Groupが空になる:

```text
Planning Groupsで linkA -> linkG のchainを作れているか確認する。
```

Planが失敗する:

```text
1. Goal Stateをrandom validではなくhomeから少しだけ動かす
2. 関節リミットが狭すぎないか確認する
3. Self-Collision Matrixを再生成する
```

ロボットが表示されない:

```text
Fixed Frameが world または linkA になっているか確認する。
```

## 6. 次の段階

MoveIt + RVizで動いたら、次は2つの方向に進める。

```text
A. MoveItのまま実機に近づける
   - joint_limits.yamlを実機の安全範囲に合わせる
   - home姿勢、待機姿勢、作業姿勢を追加する
   - 実機DYNAMIXEL用のFollowJointTrajectoryブリッジを作る

B. 物理シミュレータへ進める
   - URDFにcollisionを追加する
   - 各linkにinertialを追加する
   - ros2_controlのSystemInterfaceをfake_componentsからGazebo用へ切り替える
   - gz_ros2_control またはGazebo Classic連携で動かす
```

まずの目標はAの前半、つまりMoveItが出した関節軌道を安全な関節角列として読める状態にすること。

## 7. 今回出たエラーの対処

`demo.launch.py` 起動時に次のエラーで `move_group` が落ちる場合:

```text
parameter 'robot_description_planning.joint_limits.<joint>.max_velocity' has invalid type: expected [double] got [integer]
```

生成されたMoveIt設定の `joint_limits.yaml` で、整数の `1` / `0` を小数の `1.0` / `0.0` に直す。

```sh
perl -0pi -e 's/max_velocity: 1\b/max_velocity: 1.0/g; s/max_acceleration: 0\b/max_acceleration: 0.0/g' \
  ~/arm_ws/src/robot_arm_moveit_config/config/joint_limits.yaml

cd ~/arm_ws
colcon build --packages-select robot_arm_moveit_config
source install/setup.bash
```

RVizのPlanning Groupは、Setup Assistantで作った名前を選ぶ。この設定では `RM` で生成されている場合がある。

```text
Planning Group: RM
```

`ros2_control_node` が次のような `Fast CDR` のsymbol lookup errorで落ちる場合は、ROS 2 Jazzyのaptパッケージ不整合の可能性がある。

```text
undefined symbol: _ZN8eprosima7fastcdr3Cdr9serializeEj
```

まずMoveIt本体の `joint_limits.yaml` エラーを直してから、まだ出る場合にROS 2パッケージを更新する。

```sh
sudo apt update
sudo apt upgrade
```

`Planning Group` に `arm` が出ない場合は、SRDFで別名になっている。たとえば `RM` で生成された場合は次で `arm` に変更する。

```sh
perl -0pi -e 's/<group name="RM">/<group name="arm">/g' \
  ~/arm_ws/src/robot_arm_moveit_config/config/robot_arm_fusion.srdf
```

`AddTimeOptimalParameterization` が失敗してPlanningできない場合は、関節の加速度制限を有効にする。

```sh
perl -0pi -e 's/has_acceleration_limits: false/has_acceleration_limits: true/g; s/max_acceleration: 0\.0/max_acceleration: 1.0/g' \
  ~/arm_ws/src/robot_arm_moveit_config/config/joint_limits.yaml

cd ~/arm_ws
colcon build --packages-select robot_arm_moveit_config
source install/setup.bash
```

`Plan` は通るが `Execute` が失敗し、move_groupログに次が出る場合:

```text
No action namespace specified for controller `RM_controller`
Unable to identify any set of controllers that can actuate the specified joints
```

`moveit_controllers.yaml` に `action_ns` を追加する。

```yaml
moveit_simple_controller_manager:
  controller_names:
    - RM_controller

  RM_controller:
    type: FollowJointTrajectory
    action_ns: follow_joint_trajectory
    default: true
    joints:
      - base_yaw_joint
      - shoulder_pitch_joint
      - upper_arm_roll_joint
      - elbow_pitch_joint
      - elbow_roll_joint
      - wrist_pitch_joint
```

反映:

```sh
cd ~/arm_ws
colcon build --packages-select robot_arm_moveit_config
source install/setup.bash
```

## 8. 実機OpenCRで動かす

MoveItの `FollowJointTrajectory` をOpenCRのシリアルコマンド `q <j1> <j2> <j3> <j4> <j5> <j6>` に変換するROS 2ブリッジを追加した。

```text
~/arm_ws/src/robot_arm_opencr_bridge
```

ブリッジの初期対応は次。

```text
OpenCR q order: ID1 ID2 ID3 ID4 ID5 ID6
ID1 <- base_yaw_joint       sign +
ID2 <- shoulder_pitch_joint sign +
ID3 <- elbow_pitch_joint    sign -
ID4 <- elbow_roll_joint     sign +
ID5 <- wrist_pitch_joint    sign -
ID6 <- upper_arm_roll_joint sign +
sh
cd ~/arm_ws
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 launch robot_arm_opencr_bridge opencr_moveit.launch.py execute:=false
```

実機へ送る前に、別端末でOpenCRが見えているか確認する。

```sh
ls /dev/ttyACM* /dev/ttyUSB* 2>/dev/null
```

実機送信する場合:

```sh
cd ~/arm_ws
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 launch robot_arm_opencr_bridge opencr_moveit.launch.py \
  execute:=true \
  port:=/dev/ttyACM0 \
  max_step_deg:=5.0
```

最初の実機テストでは、RViz上で1関節だけ数度動く小さい目標を作る。`random valid` は使わない。

安全メモ:

```text
- 非常停止またはすぐ電源を切れる状態で行う
- 周囲に干渉物を置かない
- 最初は max_step_deg:=5.0 以下にする
- OpenCR側の JOINT_LIMIT_MIN_DEG / JOINT_LIMIT_MAX_DEG も実機に合わせる
- 動作方向が逆の軸があれば opencr_moveit.launch.py の motor_signs を反転する
```

`Package 'robot_arm_opencr_bridge' not found` が出る場合は、`arm_ws` のinstall環境が端末にsourceされていない。

```sh
cd ~/arm_ws
source /opt/ros/jazzy/setup.bash
colcon build --packages-select robot_arm_opencr_bridge robot_arm_moveit_config
source ~/arm_ws/install/setup.bash
ros2 pkg list | grep robot_arm_opencr_bridge
```

`robot_arm_opencr_bridge` が表示されてからlaunchする。

実機送信で最後まで `q ...` が出ているのにRViz側が `TIMED_OUT` になる場合は、OpenCRブリッジの送信遅延がMoveItの予定軌道時間を超えている可能性がある。ブリッジ側は `command_delay` のデフォルトを `0.0` にしている。古いビルドを使っている場合は再ビルドする。

```sh
cd ~/arm_ws
colcon build --packages-select robot_arm_opencr_bridge
source install/setup.bash
```

起動時に明示する場合:

```sh
ros2 launch robot_arm_opencr_bridge opencr_moveit.launch.py \
  execute:=true \
  port:=/dev/ttyACM0 \
  max_step_deg:=5.0
```

実機の動きが極端に遅い場合は、OpenCR側の `q` コマンド処理が毎点で重くなっていないか確認する。現在のファームでは次の高速化を入れている。

```text
- q軌道中は毎点のtorqueOff / mode設定 / torqueOnをしない
- q軌道中は毎点の長いUSBシリアルログを出さない
- q軌道中は準備済みモータへの毎点pingを省く
- DYNAMIXEL profile velocity=80, acceleration=20
```

OpenCRへ反映するにはアップロードする。

```sh
cd ~/Documents/robot_arm/robot_arm
make build
make upload PORT=/dev/ttyACM1
```

アップロード後はポート番号が `/dev/ttyACM0` から `/dev/ttyACM1` へ変わることがある。実機起動時は現在見えているポートを使う。

```sh
ls /dev/ttyACM* /dev/ttyUSB* 2>/dev/null
```

実機のGoal Joint値と実機の動きが合っていて、RViz上のビジュアルだけ角度が大きく見える場合は、OpenCRブリッジ側ではなくURDF/Fusionモデル側を疑う。

確認する順番:

```text
1. RViz MotionPlanning の Goal Joint 値
2. OpenCR bridge が出す q コマンド角度
3. 実機の実際の動き
4. RVizのロボットビジュアル
```

1-3が合っていて4だけ大きく動いて見えるなら、`motor_signs` や実機送信角度は触らない。見るべき候補は次。

```text
- URDFのjoint axis
- joint originの位置
- Fusionメッシュのlink割り当て
- 回転するべきリンクと固定されるべきリンクの分割
- RVizで見ているStart/Goal/Planned Path表示の混同
```


OpenCR bridge current mapping:

```text
ID1 <- base_yaw_joint       sign +
ID2 <- shoulder_pitch_joint sign +
ID3 <- upper_arm_roll_joint sign -
ID4 <- elbow_pitch_joint    sign +
ID5 <- elbow_roll_joint     sign -
ID6 <- wrist_pitch_joint    sign -
```
