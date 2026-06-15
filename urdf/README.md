# URDF Models

## `robot_arm_simple_3axis.urdf`

現在の順運動学確認用の簡易6軸URDF。

このモデルは、まず位置計算を切り分けるために次の6軸を表す。

- ID 1: ベース旋回
- ID 2: 肩ピッチ
- ID 6: 上腕ロール
- ID 3: 肘ピッチ
- ID 4: 肘ロール
- ID 5: 手首ピッチ

寸法は次の値を使う。

| 記号 | 値 |
| --- | --- |
| `L0` | `0.106 m` |
| `L1` | `0.111 m` |
| `L2` | `0.070 m` |
| `L3` | `0.080 m` |

座標系:

- `X`: アーム正面方向
- `Y`: 左方向
- `Z`: 上方向

ゼロ姿勢:

- 肩から肘までのリンクは `+Z` 方向を向く。
- 肘から手首基準点までのリンクは `+X` 方向を向く。
- `tool_tip_link` も `+X` 方向へ伸びる。

関節角の対応:

| URDF joint | モータID | 対応 |
| --- | --- | --- |
| `base_yaw_joint` | ID 1 | `+ID1` |
| `shoulder_pitch_joint` | ID 2 | `+ID2` |
| `upper_arm_roll_joint` | ID 6 | `-ID6` |
| `elbow_pitch_joint` | ID 3 | `-ID3` |
| `elbow_roll_joint` | ID 4 | `-ID4` |
| `wrist_pitch_joint` | ID 5 | `+ID5` |

注意:

- これは厳密な実機モデルではない。
- 肘ピッチ軸と肘ロール軸のオフセットはまだ近似。
- 上腕ロール、肘ロール、手首ピッチ、エンドエフェクタ姿勢はまだ近似。
- `L3=80 mm` は、ID4以降の可視化用リンクとして `30 mm + 50 mm` に分割している。

RVizなどで見る場合は、このURDFを `robot_description` に読み込む。

## PyBulletで見る

このプロジェクトでは、PyBullet用に `.conda-pybullet` 環境を使う。

```sh
make view-urdf
```

PyBulletウィンドウ内では、マウスで視点を変更できる。

- ドラッグ: 視点回転
- ホイール: ズーム
- 右ドラッグ、または環境によっては修飾キー付きドラッグ: パン

viewerには ID 1からID 6の関節スライダーも表示される。スライダーの角度は実機のモータ角と同じ向きで入力する。

初期カメラを指定する場合:

```sh
.conda-pybullet/bin/python tools/view_urdf_pybullet.py --yaw 90 --pitch -20 --distance 0.8
```

カメラ注視点を変える場合:

```sh
.conda-pybullet/bin/python tools/view_urdf_pybullet.py --target-x 0.08 --target-y 0 --target-z 0.18
```

静止画を出力する場合:

```sh
make render-urdf
```

出力先:

```text
outputs/robot_arm_simple_6axis.png
```

角度を指定して開く場合:

```sh
.conda-pybullet/bin/python tools/view_urdf_pybullet.py --j1 0 --j2 10 --j3 -10 --j4 20 --j5 15 --j6 30
```

角度指定はモータ角のまま入力する。スクリプト内で次の対応に変換している。

```text
base_yaw_joint = +ID1
shoulder_pitch_joint = +ID2
upper_arm_roll_joint = -ID6
elbow_pitch_joint = -ID3
elbow_roll_joint = -ID4
wrist_pitch_joint = +ID5
```
