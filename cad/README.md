# CAD Data

Fusionアセンブリ由来のURDFを作るためのCAD置き場。

## 置き場所

### Fusion元データ

Fusionのアセンブリデータはここに置く。

```text
cad/fusion/
```

推奨ファイル名:

```text
cad/fusion/robot_arm.f3z
```

`.f3z` が難しい場合:

```text
cad/fusion/robot_arm.f3d
```

### Fusionから書き出したメッシュ

linkごとのSTL/OBJを書き出す場合はここに置く。

```text
cad/exports/fusion_meshes/
```

例:

```text
cad/exports/fusion_meshes/base_link.stl
cad/exports/fusion_meshes/base_yaw_link.stl
cad/exports/fusion_meshes/shoulder_pitch_link.stl
```

### 関節表

関節中心、回転軸、親link、子linkを表で渡す場合はここに置く。

```text
cad/exports/fusion_tables/joints.csv
```

## 最初に欲しいもの

まずは次のファイルだけでよい。

```text
cad/fusion/robot_arm.f3z
```

アップロード後、このファイルをもとにFusion由来URDFを作る。

## 受け取り済み

現在のFusion Archive:

```text
cad/fusion/robotarm_assembly.f3z
```

展開先:

```text
cad/fusion/extracted/
```

部品対応表:

```text
cad/fusion/extracted/fusion_file_map.csv
```

## 次にやること

`.f3z` の中身はFusion独自形式のため、URDF用のSTLと関節情報はFusion上で書き出す。

Fusion 360で `robotarm_assembly.f3z` を開き、次のスクリプトを実行する。

```text
tools/fusion360_export_urdf_data.py
```

出力先:

```text
cad/exports/fusion_meshes/
cad/exports/fusion_tables/
```

スクリプト実行後に確認するファイル:

```text
cad/exports/fusion_tables/occurrences.csv
cad/exports/fusion_tables/joints_template.csv
```
