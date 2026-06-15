# Manual Joint Tuning

可動URDFの動きを手動で合わせるためのメモ。

## 1. 部品をどのリンクで動かすか

編集するCSV:

```text
cad/exports/fusion_tables/link_assignment_template.csv
```

`link_name` に、その部品が属するリンク名を入れる。

使えるリンク名:

```text
base_link
base_yaw_link
shoulder_pitch_link
upper_arm_link
forearm_link
elbow_roll_link
wrist_pitch_link
```

判断:

- 2軸で一緒に動く部品: `shoulder_pitch_link`
- 3軸で一緒に動く部品: `upper_arm_link`
- 6軸で一緒に動く部品: `forearm_link`
- 4軸で一緒に動く部品: `elbow_roll_link`
- 5軸で一緒に動く部品: `wrist_pitch_link`

## 2. 関節の回転中心と軸

編集するCSV:

```text
cad/exports/fusion_tables/joints_template.csv
```

重要な列:

```csv
joint_name,parent_link,child_link,motor_id,axis_x,axis_y,axis_z,origin_x_mm,origin_y_mm,origin_z_mm,sign
```

- `axis_x/y/z`: 親リンク座標系で見た回転軸
- `origin_x/y/z_mm`: CADワールド座標で見た関節中心 [mm]
- `sign`: 実機モータ角との符号対応

現在の対応:

```text
ID1: base_yaw_joint
ID2: shoulder_pitch_joint
ID3: elbow_pitch_joint
ID6: upper_arm_roll_joint
ID4: elbow_roll_joint
ID5: wrist_pitch_joint
```

## 3. 反映と確認

```sh
make fusion-jointed
make view-fusion-urdf
```

PNGで見る場合:

```sh
make render-fusion-urdf
```

角度を指定して確認する例:

```sh
.venv-ubuntu/bin/python tools/render_urdf_pybullet.py --urdf urdf/robot_arm_fusion.urdf --output outputs/check_j4.png --j4 45
.venv-ubuntu/bin/python tools/render_urdf_pybullet.py --urdf urdf/robot_arm_fusion.urdf --output outputs/check_j5.png --j5 45
.venv-ubuntu/bin/python tools/render_urdf_pybullet.py --urdf urdf/robot_arm_fusion.urdf --output outputs/check_j6.png --j6 45
```
