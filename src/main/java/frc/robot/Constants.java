package frc.robot;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;

/**
 * ロボット全体で共有する定数クラス。
 * ハードウェアIDや制御パラメータはここに集約し、他クラスから参照する。
 */
public final class Constants {

    private Constants() {}

    // ─────────────────────────────────────────────
    //  チーム番号
    // ─────────────────────────────────────────────
    public static final int TEAM_NUMBER = 9999;

    // ─────────────────────────────────────────────
    //  Drivetrain — CAN IDs
    //  モジュール順: FL(前左), FR(前右), BL(後左), BR(後右)
    // ─────────────────────────────────────────────
    public static final class DrivetrainIDs {
        // Drive モータ (TalonFX / CTRE)
        public static final int FL_DRIVE = 1;
        public static final int FR_DRIVE = 2;
        public static final int BL_DRIVE = 3;
        public static final int BR_DRIVE = 4;

        // Steer モータ (TalonFX / CTRE)
        public static final int FL_STEER = 5;
        public static final int FR_STEER = 6;
        public static final int BL_STEER = 7;
        public static final int BR_STEER = 8;

        // CANcoder (絶対エンコーダ)
        public static final int FL_ENCODER = 9;
        public static final int FR_ENCODER = 10;
        public static final int BL_ENCODER = 11;
        public static final int BR_ENCODER = 12;

        // Pigeon 2 (IMU)
        public static final int PIGEON = 13;
    }

    // ─────────────────────────────────────────────
    //  Drivetrain — 物理パラメータ
    // ─────────────────────────────────────────────
    public static final class DrivetrainPhysics {
        // ホイールベース (m) — FL→FR 間の横幅 (SDS MK4n 645mm フレーム)
        public static final double TRACK_WIDTH = 0.3225;
        // ホイールベース (m) — FL→BL 間の前後長
        public static final double WHEEL_BASE  = 0.3225;

        // ドライブ半径 = コーナーまでの距離 (AutoBuilder 用)
        public static final double DRIVE_RADIUS =
            Math.hypot(TRACK_WIDTH / 2.0, WHEEL_BASE / 2.0);

        // ホイール直径 (m)
        public static final double WHEEL_DIAMETER = Units.inchesToMeters(4.0);

        // ドライブギア比 — SDS MK4n L2
        public static final double DRIVE_GEAR_RATIO = 5.90;

        // ステアギア比 — SDS MK4n
        public static final double STEER_GEAR_RATIO = 18.75;

        // 最大速度 (m/s) / 最大角速度 (rad/s)
        public static final double MAX_SPEED     = 4.5;
        public static final double MAX_ANGULAR   = 2.0 * Math.PI;
    }

    // ─────────────────────────────────────────────
    //  Shooter — CAN IDs & 制御
    // ─────────────────────────────────────────────
    public static final class ShooterConstants {
        public static final int LEFT_MOTOR_ID  = 20;
        public static final int RIGHT_MOTOR_ID = 21;

        // 通常射出の目標 RPM（距離テーブルで上書き）
        public static final double DEFAULT_LEFT_RPM  = 3000.0;
        public static final double DEFAULT_RIGHT_RPM = 3000.0;

        // ── 2段階 RPM ──────────────────────────────────
        // ~3 m: 近距離固定 RPM
        public static final double RPM_CLOSE = 2800.0;
        // 3 m~: 遠距離固定 RPM
        public static final double RPM_FAR   = 4200.0;
        // キャリーショット (NEUTRAL ZONE → 自陣パス) 用 RPM
        public static final double RPM_CARRY = 3500.0;
        // 近距離/遠距離切り替え閾値 (m)
        public static final double RPM_THRESHOLD_M = 3.0;

        // RPM 許容誤差
        public static final double RPM_TOLERANCE = 100.0;

        // Flywheel debounce 時間 (s) — この時間 RPM 範囲内を維持して初めて READY とする
        public static final double FLYWHEEL_DEBOUNCE_S = 0.10;

        // ホイール直径 (m) — 飛行時間推定に使用
        public static final double FLYWHEEL_DIAMETER_M = Units.inchesToMeters(4.0);

        // フィードフォワード係数 kV
        public static final double KV = 0.00022;
        // PID
        public static final double KP = 0.0004;
        public static final double KI = 0.0;
        public static final double KD = 0.0;
    }

    // ─────────────────────────────────────────────
    //  Feeder — CAN IDs & 制御
    // ─────────────────────────────────────────────
    public static final class FeederConstants {
        public static final int MOTOR_ID = 22;

        public static final double FEED_SPEED  =  0.8;  // 0.0〜1.0
        public static final double EJECT_SPEED = -0.5;
    }

    // ─────────────────────────────────────────────
    //  Intake — CAN IDs & 制御
    // ─────────────────────────────────────────────
    public static final class IntakeConstants {
        public static final int MOTOR_ID = 23;

        public static final double INTAKE_SPEED  =  0.9;
        public static final double EJECT_SPEED   = -0.5;
    }

    // ─────────────────────────────────────────────
    //  Extension — CAN IDs & 制御
    // ─────────────────────────────────────────────
    public static final class ExtensionConstants {
        public static final int MOTOR_ID = 24;

        // ソフトリミット (回転数)
        public static final double SOFT_LIMIT_FWD = 50.0;
        public static final double SOFT_LIMIT_REV = 0.0;

        // 延伸目標位置 (回転数)
        public static final double EXTEND_POSITION  = 48.0;
        public static final double RETRACT_POSITION = 0.0;

        // PID
        public static final double KP = 0.1;
        public static final double KI = 0.0;
        public static final double KD = 0.0;

        // 位置許容誤差 (回転数)
        public static final double POSITION_TOLERANCE = 1.0;

        // ThroughBoreEncoder DIO ポート番号
        public static final int THROUGH_BORE_DIO_PORT = 0;
    }

    // ─────────────────────────────────────────────
    //  Localization — PoseEstimator ノイズ定数
    // ─────────────────────────────────────────────
    public static final class LocalizationConstants {
        // オドメトリ標準偏差 [x(m), y(m), θ(rad)]
        public static final double[] ODOMETRY_STD_DEVS = {0.05, 0.05, 0.01};

        // Vision 基準標準偏差（距離補正前） [x(m), y(m), θ(rad)]
        public static final double[] VISION_BASE_STD_DEVS = {0.5, 0.5, Math.toRadians(5)};

        // 距離スケーリング係数 — dist/SCALE が誤差倍率に加算される
        public static final double VISION_DIST_SCALE = 4.0;

        // Vision 測定を破棄する最大ジャンプ距離 (m)
        public static final double MAX_POSE_JUMP = 1.0;
    }

    // ─────────────────────────────────────────────
    //  Vision — Limelight 設定
    // ─────────────────────────────────────────────
    public static final class VisionConstants {
        public static final String LIMELIGHT_NAME = "limelight";

        // カメラマウント位置 (ロボット中心から) [前(m), 左(m), 高さ(m)]
        public static final double CAM_FORWARD = Units.inchesToMeters(10.0);
        public static final double CAM_LEFT    = 0.0;
        public static final double CAM_HEIGHT  = Units.inchesToMeters(20.0);

        // カメラ仰角 (deg)
        public static final double CAM_PITCH_DEG = 20.0;

        // Limelight 伝送遅延の初期推定値 (s) — NT から実測値に上書き
        public static final double LATENCY_FALLBACK_S = 0.040;
    }

    // ─────────────────────────────────────────────
    //  フィールド定数 (WPIBlue 座標系)
    // ─────────────────────────────────────────────
    public static final class FieldConstants {
        // フィールドサイズ (m)
        public static final double FIELD_LENGTH = 17.548;
        public static final double FIELD_WIDTH  = 8.211;

        // Speaker 開口部中央座標 (Blue Alliance 基準)
        public static final Translation2d SPEAKER_BLUE =
            new Translation2d(0.0, 5.547);
        public static final Translation2d SPEAKER_RED =
            new Translation2d(FIELD_LENGTH, 5.547);

        // HUB 中央座標 — REBUILT 2025-26: フィールド中央の固定構造物
        // ※ 公式フィールド図面で実測後に更新すること
        public static final Translation2d HUB_CENTER =
            new Translation2d(FIELD_LENGTH / 2.0, FIELD_WIDTH / 2.0);

        // HUB 開口部の高さ (m)
        public static final double HUB_HEIGHT     = 2.64;

        // シューター射出口の高さ (m)
        public static final double SHOOTER_HEIGHT  = Units.inchesToMeters(24.0);

        // NEUTRAL ZONE X 範囲 (WPIBlue 座標系) — 要図面確認
        public static final double NEUTRAL_ZONE_MIN_X = FIELD_LENGTH / 2.0 - 1.5;
        public static final double NEUTRAL_ZONE_MAX_X = FIELD_LENGTH / 2.0 + 1.5;

        // Landing Zone 中央座標 (キャリーショット着地目標)
        // ※ 公式フィールド図面で実測後に更新すること
        public static final Translation2d LANDING_ZONE_BLUE =
            new Translation2d(2.5, FIELD_WIDTH / 2.0);
        public static final Translation2d LANDING_ZONE_RED  =
            new Translation2d(FIELD_LENGTH - 2.5, FIELD_WIDTH / 2.0);
    }

    // ─────────────────────────────────────────────
    //  Hood — 発射角度調整機構
    // ─────────────────────────────────────────────
    public static final class HoodConstants {
        public static final int MOTOR_ID = 25;

        // 可動範囲 (deg)
        public static final double MIN_ANGLE_DEG  = 20.0;
        public static final double MAX_ANGLE_DEG  = 70.0;

        // 格納位置 (deg)
        public static final double STOW_ANGLE_DEG = 25.0;

        // ギア比 — KrakenX60 → Hood 10:1
        public static final double GEAR_RATIO     = 10.0;

        // ソフトリミット (モータ回転数換算)
        public static final double SOFT_LIMIT_FWD = MAX_ANGLE_DEG * GEAR_RATIO / 360.0;
        public static final double SOFT_LIMIT_REV = MIN_ANGLE_DEG * GEAR_RATIO / 360.0;

        // PID
        public static final double KP = 0.08;
        public static final double KI = 0.0;
        public static final double KD = 0.002;

        // 角度許容誤差 (deg)
        public static final double ANGLE_TOLERANCE_DEG = 1.5;
    }

    // ─────────────────────────────────────────────
    //  Spindexer — CAN ID & 制御
    // ─────────────────────────────────────────────
    public static final class SpindexerConstants {
        public static final int    MOTOR_ID          = 26;
        public static final double SPIN_SPEED        = 0.5;   // duty-cycle 0〜1
        public static final double REVERSE_SPEED     = -0.3;
        // Feeder 起動後この時間経過でSpindexerを起動する (s)
        public static final double SPINDEXER_DELAY_S = 0.15;
    }

    // ─────────────────────────────────────────────
    //  CarryShoot — NEUTRAL ZONE キャリー射出
    // ─────────────────────────────────────────────
    public static final class CarryConstants {
        // 低軌道パス用 Hood 角度 (deg) — 鋭角で水平に近い放物線
        public static final double CARRY_HOOD_ANGLE_DEG = 35.0;

        // Heading PID 許容誤差 (deg)
        public static final double HEADING_TOLERANCE_DEG = 3.0;
    }

    // ─────────────────────────────────────────────
    //  コントローラ — ポート番号
    // ─────────────────────────────────────────────
    public static final class OperatorConstants {
        public static final int DRIVER_CONTROLLER_PORT   = 0;
        public static final int OPERATOR_CONTROLLER_PORT = 1;

        // スティックのデッドバンド
        public static final double JOYSTICK_DEADBAND = 0.08;
    }
}
