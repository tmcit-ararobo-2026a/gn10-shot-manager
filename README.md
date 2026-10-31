# gn10_shot_manager

高専ロボコン2026ロボット用 **投擲目標ポイント微調整エディタ (TUI)** 兼 **射出試行同期レコーダー (Trial Recorder)** パッケージ。

---

## 概要

- **Shooting GUI Editor (`shooting_gui`)**: 本番フィールド図（Red/Blueゾーン、固定バケツ・机・旗等のターゲットオブジェクト）上で各ターゲット位置（X, Y, Yaw）と目標射出速度（V_tgt）をクリック・ドラッグ配置および微調整できる2D GUIツール。専用データリポジトリ（`data/shooting-points`）のバージョン管理と連携し、非同期・軽量保存でGUIをブロックすることなくメモリ反映・YAML保存・Git自動コミットまで完結します。
- **Shooting Editor (TUI) (`shooting_editor`)**: SSH接続端末上でも軽量かつ確実に動作するターミナルCUIエディタ。
- **Trial Recorder**: ロボットの射出カウンタトピック (`/robot/feedback/shot_count`) を監視し、射出トリガ発火時に同一時間軸でロボットの各種センサ（ベルト速度、電圧、電流、目標値）と RealSense による雑巾追跡位置 (`/cloth/position`) をサンプリング・同期記録（`data/shot-trial-log/trials.jsonl`）。
- **Trial Viewer**: 記録された各試行ログから着弾精度やターゲットごとの統計（平均誤差、標準偏差、ピーク電流等）を表示。

---

## データリポジトリ構成 (`data/shooting-points`)

位置調整データ（`shooting_points.yaml`）はコードリポジトリへの混入を防ぎ履歴管理を独立させるため、`data/shot-trial-log` と同様に別リポジトリ（`data/shooting-points`）でバージョン管理されます。
- **保存先**: `~/ros2_ws/data/shooting-points/shooting_points.yaml` (シンボリックリンク `shooting_point.yaml` 経由でもアクセス可能)
- **更新連携**: `ros2_ws` の `setup.sh` / `update.sh` (`private_2026a.repos`) で自動管理されます。
- **フォールバック**: もし `data/shooting-points` が存在しない場合は、従来通り `src/gn10-navigation/config/shooting_points.yaml` または share ディレクトリを自動フォールバック探索します。

---

## 主な機能と使い方

### 1. Shooting GUI Editor (`shooting_gui`)

GUI画面から直感的に本番フィールドの射出目標地点を調整・登録します。

```bash
ros2 run gn10_shot_manager shooting_gui
```

- **フィールド図上操作**:
  - **左クリック / ドラッグ**: 選択中ターゲットの大まかな位置を決定・移動
  - **右クリックドラッグ**: 射出姿勢（Yaw角）を直感的に回転調整
  - **相手ゾーン視覚化**: Red Zone (Y < 0) / Blue Zone (Y > 0) とターゲットオブジェクト（固定バケツ①②③、机1・2、旗）が表示され、相手オブジェクト前での位置合わせを容易に確認可能
- **微調整パネル**:
  - X, Y (`±5cm`, `±0.005m`), Yaw (`±5°`, `±1°`), Target Velocity (`±0.5m/s`, `±0.1m/s`), Memo の数値直接編集
- **アクション**:
  - **💾 Save & Commit**: `/target_manager/update_point`（メモリ更新）→ `/target_manager/save_points`（YAML保存）→ Git自動コミット
  - **🔄 Reload YAML**: ディスクの `shooting_points.yaml` から再読込
  - **🎯 Test Target**: 選択中ターゲットIDを `/robot/target_id` へ配信

---

### 2. Shooting Editor (TUI) (`shooting_editor`)

SSH接続端末などのターミナル環境から起動:
```bash
ros2 run gn10_shot_manager shooting_editor
```

#### キーバインド一覧:
| キー | 操作内容 |
|---|---|
| `↑` / `↓` (`k` / `j`) | ターゲット行の選択 (Target 0～7) |
| `←` / `→` (`h` / `l`) | 選択中フィールドの微小調整 (`±0.005m`, `±0.01rad`) |
| `+` / `-` | 選択中フィールドの微小加算 / 減算 |
| `>` / `<` | 選択中フィールドの粗調整 (`±0.025m`) |
| `Tab` / `1`〜`5` | 調整フィールドの切替 (`1: X`, `2: Y`, `3: Yaw`, `4: V_tgt`, `5: Memo`) |
| `Enter` / `e` | 直接数値・テキスト入力編集 |
| `s` | **保存 & Gitコミット** (`UpdatePoint` → `SavePoints` → `git commit`) |
| `r` | YAMLファイルからの再読み込み |
| `t` | 現在選択中の TargetId を `/robot/target_id` へ配信（目標送信） |
| `q` / `ESC` | 終了（未保存変更がある場合は確認プロンプト表示） |

---

### 3. Trial Recorder (`trial_recorder`)

バックグラウンドまたは別ターミナルで起動:
```bash
ros2 run gn10_shot_manager trial_recorder
# または
ros2 launch gn10_shot_manager shot_manager.launch.py
```

- **トリガ**: `/robot/feedback/shot_count` のインクリメント（原点取り時の0リセットは無視）
- **保存先**: `~/ros2_ws/data/shot-trial-log/trials.jsonl`（専用リポジトリ `shot-trial-log` 内）
- **リポジトリ管理連携**:
  - `ros2_ws` の `setup.sh` / `update.sh` (`private_2026a.repos`) により、`data/shot-trial-log` は自動でクローン & pull されます。
  - **セッション要約コミット**: 射出頻度が高くてもコミット履歴の見通しを損なわないよう、毎射出ごとではなくノード終了時にそのセッションの試行数・日時・ターゲット内訳を要約した1つのコミット（例: `log: trial session summary (2026-10-28 20:20 - 21:05, 15 shots)`）を自動生成します。
- **記録データ**:
  - `shot_id`, `timestamp`, `target` (id, name, x, y, yaw, target_velocity, memo)
  - `summary`: 最大速度、最大電流、平均電圧
  - `landing_position`: 雑巾着弾座標
  - `error`: 着弾誤差（dx, dy, distance）
  - `samples`: 50Hz (20ms刻み) で同期された時系列データ

---

### 3. Trial Viewer (`trial_viewer`)

```bash
# 全体サマリー & ターゲット別着弾精度統計
ros2 run gn10_shot_manager trial_viewer

# 最新の試行の詳細データ表示
ros2 run gn10_shot_manager trial_viewer --last

# 特定の Shot ID (例: 1) の時系列データ表示
ros2 run gn10_shot_manager trial_viewer --shot 1
```

---

## 連携トピック・サービス仕様

| 名前 | 型 | 種別 | 説明 |
|---|---|---|---|
| `/target_manager/update_point` | `gn10_navigation/srv/UpdatePoint` | Client | メモリ上ポイント更新 |
| `/target_manager/save_points` | `std_srvs/srv/Trigger` | Client | ファイル永続化保存 |
| `/robot/feedback/shot_count` | `std_msgs/msg/UInt8` | Sub/Pub | メイン基板の射出回数カウンタ（原点取り0、射出命令でインクリメント） |
| `/robot/feedback/belt_launcher/*` | `std_msgs/msg/Float32` | Sub | 実ベルト速度 (velocity)、連続指令値 (command)、リリース目標値 (target)、電圧・電流 |
| `/robot/feedback/drive_*` | `std_msgs/msg/Float32` | Sub | 走行バッテリー電圧・電流 |
| `/cloth/position` | `geometry_msgs/msg/PointStamped` | Sub | 雑巾の3D位置 (RealSense追跡) |
| `/cloth/measured_path` | `nav_msgs/msg/Path` | Sub | 雑巾の飛行軌跡パス |

