# FusionアセンブリからURDFを作る方針

手計測で作ったURDFは、関節方向やゼロ姿勢を確認するための仮モデルとして残す。今後はFusionのアセンブリを正として、リンク形状、関節位置、関節軸をURDFへ反映する。

## 方針

Fusion側で次の情報を正確に持たせる。

- リンクごとの剛体グループ
- 各関節の回転中心
- 各関節の回転軸
- 全軸 `0 deg` の基準姿勢
- エンドエフェクタのツール先端位置

URDF側では、Fusionの情報を使って次を作る。

- `link`
  - Fusionの剛体グループに対応
  - 表示用メッシュはFusionから出力したSTL/OBJを使う

- `joint`
  - Fusionの関節中心と関節軸に対応
  - `revolute` jointとして定義
  - DYNAMIXELのモータIDと符号を対応させる

## 推奨するFusion側の整理

Fusionのブラウザ上で、URDFのlinkにしたい単位へコンポーネントを分ける。

例:

```text
base_link
base_yaw_link
shoulder_pitch_link
upper_arm_roll_link
upper_arm_link
forearm_link
elbow_roll_link
wrist_pitch_link
tool_link
```

ポイント:

- 1つのURDF linkは、動かない部品のまとまりにする。
- モータ、ホーン、3Dプリント部品、ネジなどが同じ関節で一体に動くなら同じlinkへまとめる。
- 関節で相対回転する部分は別linkに分ける。
- Fusionのジョイント原点は、DYNAMIXELの回転軸中心に合わせる。

## 出力してほしいデータ

Fusionから次のどちらかを用意する。

### 方法A: アセンブリファイル

```text
robot_arm.f3z
```

複数コンポーネントを含む場合は、`.f3d` より `.f3z` の方が扱いやすい。

### 方法B: メッシュと関節表

```text
fusion_export/
  meshes/
    base_link.stl
    base_yaw_link.stl
    shoulder_pitch_link.stl
    upper_arm_roll_link.stl
    upper_arm_link.stl
    forearm_link.stl
    elbow_roll_link.stl
    wrist_pitch_link.stl
    tool_link.stl
  joints.csv
```

`joints.csv` の列:

```csv
joint_name,parent_link,child_link,motor_id,axis_x,axis_y,axis_z,origin_x_mm,origin_y_mm,origin_z_mm,sign
base_yaw_joint,base_link,base_yaw_link,1,0,0,1,0,0,0,1
shoulder_pitch_joint,base_yaw_link,shoulder_pitch_link,2,0,1,0,0,0,106,1
upper_arm_roll_joint,shoulder_pitch_link,upper_arm_link,6,0,0,-1,0,0,124,-1
elbow_pitch_joint,upper_arm_link,forearm_link,3,0,1,0,0,0,217,-1
elbow_roll_joint,forearm_link,elbow_roll_link,4,-1,0,0,70,0,0,-1
wrist_pitch_joint,elbow_roll_link,wrist_pitch_link,5,0,1,0,30,0,0,1
```

この表は例であり、実際の原点と軸はFusionの座標から取る。

## 今の仮URDFから引き継ぐ情報

現在の実機確認から、モータ符号の候補は次。

| motor ID | 関節候補 | 符号 |
| --- | --- | --- |
| 1 | ベース旋回 | `+1` |
| 2 | 肩ピッチ | `+1` |
| 3 | 肘ピッチ | `-1` |
| 4 | 肘ロール | `-1` |
| 5 | 手首ピッチ | `+1` |
| 6 | 上腕ロール | `-1` |

ID 6 は手先ロールではなく、実機画像に基づいて土台に近い上腕側ロールとして扱う。

## 移行手順

1. Fusionアセンブリを全軸 `0 deg` の姿勢にする。
2. URDF link単位でコンポーネントを整理する。
3. 各関節の回転中心と軸をFusion上で確認する。
4. 各linkの表示メッシュを出力する。
5. 関節表を作る。
6. 生成URDFをPyBulletで表示する。
7. 実機の `0` コマンド姿勢とシミュレータの0姿勢を比較する。
8. `j <id> 10` と同じ角度をPyBulletへ入れて、向きが合うか確認する。
9. 合わない軸は `axis` または `sign` を修正する。

## 現在の仮URDFの扱い

`urdf/robot_arm_simple_3axis.urdf` は、Fusion由来URDFができるまでの確認用として残す。

Fusion由来URDFは別名で作る。

```text
urdf/robot_arm_fusion.urdf
```

これにより、仮モデルとFusionモデルを比較できる。
