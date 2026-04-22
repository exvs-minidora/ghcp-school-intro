# FRC2026 Robot Code — 設計計画

## 概要

| 項目 | 内容 |
|------|------|
| フレームワーク | WPILib Java — Command Based |
| DriveBase | Swerve Drive（YAGSL） |
| Autonomous | PathPlanner |
| メカニズム | Shooter / Feeder / Extend / Intake |
| モータ/制御 | REV (SPARK MAX/Flex) + CTRE (TalonFX/Pigeon2) 混在 |
| 自己位置推定 | SwerveDrivePoseEstimator（オドメトリ + IMU） |
| Vision補正 | Limelight 4（AprilTag → Kalman融合） |
| Aiming対象 | Speaker（遠距離優先） |

---

## ディレクトリ構成（予定）

```
src/main/java/frc/robot/
├── Robot.java                              # ライフサイクル・モード遷移
├── RobotContainer.java                     # 全サブシステム統合・バインド・Auto選択
├── Constants.java                          # CAN ID / 制御定数 / ノイズ定数
│
├── subsystems/
│   ├── drivetrain/
│   │   └── DrivebaseSubsystem.java         # YAGSL統合・走行API
│   ├── shooter/
│   │   └── ShooterSubsystem.java           # 射出モータ（速度制御）
│   ├── feeder/
│   │   └── FeederSubsystem.java            # 弾供給制御
│   ├── intake/
│   │   └── IntakeSubsystem.java            # 吸入制御
│   └── extension/
│       └── ExtensionSubsystem.java         # 伸展制御
│
├── localization/
│   └── PoseEstimatorSubsystem.java         # SwerveDrivePoseEstimator運用
│
├── vision/
│   ├── LimelightSubsystem.java             # AprilTag観測取得・正規化
│   └── VisionFusion.java                   # ゲーティング・タイムスタンプ補正・融合注入
│
├── commands/
│   ├── drive/
│   │   ├── TeleopSwerveCommand.java        # 通常操縦（フィールド/ロボット基準切替）
│   │   └── LockWheelsCommand.java          # Xフォーメーション停止
│   ├── shooter/
│   │   └── ShootCommand.java
│   ├── feeder/
│   │   └── FeedCommand.java
│   ├── intake/
│   │   └── IntakeCommand.java
│   ├── extension/
│   │   ├── ExtendCommand.java
│   │   └── RetractCommand.java
│   └── aim/
│       └── AimSpeakerCommand.java          # Pose+IMU → Speaker相対方位計算
│
└── autonomous/
    └── AutoFactory.java                    # AutoBuilder初期化・NamedCommands登録

src/main/deploy/
└── pathplanner/
    └── paths/                              # PathPlanner 経路JSON
```

---

## 実装フェーズ

### Phase 0 — プロジェクト初期化

1. WPILib Java Command Based 雛形を作成する。  
2. `build.gradle` にREV / CTRE / YAGSL / PathPlannerのベンダー依存を追加する。  
3. パッケージ構造を確定し、各クラスの空実装と依存注入の入口を作る。

### Phase 1 — Swerve DriveBase（以降の全機能の前提）

1. YAGSL設定（モジュール定数・ギア比・IMU接続）を実装する。  
2. フィールド基準 / ロボット基準の切替を実装する。  
3. `TeleopSwerveCommand`（デッドバンド・速度制限・姿勢補助）を実装する。  
4. ダッシュボード可視化（IMU方位・モジュール状態）でベンチ検証する。

### Phase 2 — 自己位置推定（Phase 1 依存）

1. `SwerveDrivePoseEstimator` を中心にオドメトリ更新ループを実装する。  
2. Poseリセット API（Auto開始時・テスト時）と Field2d 表示を整備する。  
3. state stdDev / vision stdDev を定数化し調整可能にする。

### Phase 3 — Limelight 4 AprilTag 融合（Phase 2 依存）

1. AprilTag観測を `VisionMeasurement` 形式へ正規化する。  
2. 測定ゲーティング（タグ妥当性・距離・急跳び除外）を実装する。  
3. タイムスタンプ補正（撮像遅延 ≈ 40 ms 考慮）を実装する。  
4. `addVisionMeasurement` でカルマン融合補正を注入する。  
5. Limelight 方針は **Megatag2 中心**で開始し、差し替え可能な抽象化を採用する（方針確定後に切替）。

> **stdDevs 動的調整（距離依存）:**  
> $$\sigma_{x,y} = \sigma_{base} \cdot \left(1 + \frac{d}{5.0}\right)$$

### Phase 4 — メカニズム（ハードウェアID確定後、Phase 3 と並列可）

1. Shooter / Feeder / Intake / Extension の各 Subsystem を実装する（REV+CTRE の差異を I/O 境界で吸収）。  
2. 電流制限・ソフトリミット・停止時挙動など安全制御を追加する。  
3. 各操作コマンドとトリガーバインドを実装する。

### Phase 5 — Aiming（Phase 2 + 3 + 4 依存）

1. 現在 Pose と IMU から Speaker までの相対方位・距離を算出する。  
2. 距離 → 目標 RPM / 角度の補間テーブルを実装する。  
3. 必要に応じて移動中補正（速度予測）を段階導入する。

### Phase 6 — Autonomous — PathPlanner（Phase 1+2 依存、4+5 統合）

1. `AutoBuilder.configureHolonomic` を Drive / Pose API に接続し、アライアンス反転を設定する。  
2. `NamedCommands` に Intake / Shoot / Extend シーケンスを登録する。  
3. 代表オート 2〜3 本を作成し、実機で追従誤差を評価する。

### Phase 7 — 統合チューニング・信頼性

1. ループ周期・CAN 負荷・NT 遅延を監視し、過負荷時フェイルセーフを追加する。  
2. Vision 喪失時はオドメトリ単独にフォールバックし、復帰時は緩やかに再融合する。  
3. Pose 誤差・タグ観測・射出成功率のログを蓄積し定数を最終調整する。

---

## 検証チェックリスト

### ビルド
- [ ] `./gradlew build` が成功する

### Phase 1 Teleop
- [ ] フィールド基準 / ロボット基準切替が正常に動作する
- [ ] 回頭追従・停止挙動が意図通りである
- [ ] ループオーバーランが発生しない

### Phase 2 オドメトリ
- [ ] 既知経路走行で位置誤差 ＜ 0.5 m
- [ ] 360° 回転後の yaw 誤差 ＜ 5°

### Phase 3 Vision 融合
- [ ] タグ視認時に Pose が収束する（誤差 ＜ 0.1 m）
- [ ] タグ遮蔽時に推定が不安定にならない
- [ ] 急跳びゲーティングが機能する
- [ ] タイムスタンプ逆行時にエラーログが出力される

### Phase 4 メカ
- [ ] 各モータが目標値に追従する
- [ ] 電流制限・ソフトリミットが動作する

### Phase 5 Aiming
- [ ] 距離別に目標 RPM / 角度が変化する
- [ ] 統計的な命中率を記録する

### Phase 6 Auto
- [ ] PathPlanner 経路追従誤差 ＜ 0.2 m
- [ ] NamedCommands が正しいタイミングで発火する
- [ ] 赤 / 青アライアンスの反転が正常に動作する

### 回帰
- [ ] Vision OFF → ON 時の再融合安定性を確認する
- [ ] IMU 再初期化後の回復を確認する
- [ ] 通信遅延時のフェイルセーフを確認する

---

## 未確定事項・検討ポイント

| 項目 | 選択肢 | 推奨 |
|------|--------|------|
| Limelight 観測方式 | A: Megatag2 固定（実装最短）/ B: 生データ推定（精度重視）/ C: 場面切替 | C（段階導入） |
| Aiming 補間方式 | A: 固定テーブル（調整容易）/ B: 回帰式（保守容易）/ C: テーブル+補正 | C |
| テスト基盤 | A: 手動ログ中心 / B: AdvantageKit 等 / C: 最小自動化＋手動 | C |

---

## Gradle 依存関係（予定）

```groovy
dependencies {
    // YAGSL
    implementation 'com.yagsl:YAGSL:2026.+'

    // PathPlanner
    implementation 'com.pathplanner.lib:PathplannerLib:2026.+'

    // REV Robotics
    implementation 'com.revrobotics.frc:REVLib:2026.+'

    // CTRE Phoenix 6
    implementation 'com.ctre.phoenix6:latest.java'
}
```
