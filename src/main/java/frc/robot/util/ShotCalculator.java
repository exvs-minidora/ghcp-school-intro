package frc.robot.util;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.Constants.FieldConstants;
import frc.robot.Constants.ShooterConstants;

import java.util.Map;
import java.util.NavigableMap;
import java.util.TreeMap;

/**
 * 距離 → Hood 角度 / Shooter RPM の最近傍ルックアップと
 * 飛行時間推定を提供するユーティリティクラス。
 *
 * <h3>テーブル設計</h3>
 * <ul>
 *   <li>距離キーは 0.5 m 刻み (1.0 〜 6.0 m)</li>
 *   <li>RPM は {@link ShooterConstants#RPM_THRESHOLD_M} を境に近距離/遠距離で 2 段階固定</li>
 *   <li>Hood 角度のみテーブル参照（最近傍）— 補間しないことで挙動を予測しやすくする</li>
 * </ul>
 *
 * <p>テーブルの値はプレースホルダ。実機試射で校正すること。
 */
public final class ShotCalculator {

    private ShotCalculator() {}

    /**
     * 距離 → Hood 角度 (deg) のルックアップテーブル。
     * キー: HUB までの水平距離 (m)、値: 最適 Hood 角度 (deg)
     */
    private static final NavigableMap<Double, Double> HOOD_TABLE = new TreeMap<>();

    static {
        // ── 近距離帯 (~3 m) ─────────────────────────────────────
        HOOD_TABLE.put(1.0, 65.0);
        HOOD_TABLE.put(1.5, 60.0);
        HOOD_TABLE.put(2.0, 54.0);
        HOOD_TABLE.put(2.5, 48.0);
        HOOD_TABLE.put(3.0, 43.0);
        // ── 遠距離帯 (3 m~) ─────────────────────────────────────
        HOOD_TABLE.put(3.5, 38.0);
        HOOD_TABLE.put(4.0, 34.0);
        HOOD_TABLE.put(4.5, 31.0);
        HOOD_TABLE.put(5.0, 28.0);
        HOOD_TABLE.put(5.5, 26.0);
        HOOD_TABLE.put(6.0, 25.0);
    }

    // ── 公開 API ──────────────────────────────────────────────

    /**
     * ロボットの現在位置から HUB に対する最適なショットパラメータを計算する。
     *
     * @param robotPose 現在の推定 Pose
     * @return {@link ShotParameters}（Hood 角度・RPM・HUB への方位角）
     */
    public static ShotParameters calculate(Pose2d robotPose) {
        Translation2d hub     = FieldConstants.HUB_CENTER;
        Translation2d robotXY = robotPose.getTranslation();

        double dist       = robotXY.getDistance(hub);
        double hoodAngle  = lookupHoodAngle(dist);
        double rpm        = selectRpm(dist);
        double headingRad = Math.atan2(hub.getY() - robotXY.getY(),
                                       hub.getX() - robotXY.getX());

        return new ShotParameters(hoodAngle, rpm, headingRad, dist);
    }

    /**
     * 簡易物理モデルで推定飛行時間を返す。
     *
     * <p>計算式: {@code t = (-vy + sqrt(vy^2 + 2*g*Δh)) / g}
     * ただし Δh = HUB_HEIGHT - SHOOTER_HEIGHT (高さ差)。
     * vy = v0 * sin(hoodAngle)。
     *
     * @param distanceM    水平距離 (m)
     * @param hoodAngleDeg Hood 角度 (deg)
     * @param rpm          Shooter RPM
     * @return 推定飛行時間 (s) — 最小 0.0
     */
    public static double estimateFlightTime(double distanceM, double hoodAngleDeg, double rpm) {
        double v0   = rpmToMps(rpm);
        double vy   = v0 * Math.sin(Math.toRadians(hoodAngleDeg));
        double dh   = FieldConstants.HUB_HEIGHT - FieldConstants.SHOOTER_HEIGHT;
        double g    = 9.80665;

        // 二次方程式: (1/2)*g*t^2 - vy*t - dh = 0 → 正根を取る
        double disc = vy * vy + 2.0 * g * dh;
        if (disc < 0.0) return 0.0;
        return (vy + Math.sqrt(disc)) / g;
    }

    // ── 内部ヘルパー ──────────────────────────────────────────

    /**
     * 指定距離に最も近いテーブルエントリの Hood 角度を返す。
     * テーブル範囲外は端値を返す。
     */
    static double lookupHoodAngle(double distanceM) {
        if (HOOD_TABLE.isEmpty()) return 45.0;

        Double lo = HOOD_TABLE.floorKey(distanceM);
        Double hi = HOOD_TABLE.ceilingKey(distanceM);

        if (lo == null) return HOOD_TABLE.firstEntry().getValue();
        if (hi == null) return HOOD_TABLE.lastEntry().getValue();

        // 最近傍を選択（距離差が小さい方）
        if (distanceM - lo <= hi - distanceM) {
            return HOOD_TABLE.get(lo);
        } else {
            return HOOD_TABLE.get(hi);
        }
    }

    /** 距離閾値に基づいて 2 段階 RPM を返す。 */
    static double selectRpm(double distanceM) {
        return distanceM < ShooterConstants.RPM_THRESHOLD_M
            ? ShooterConstants.RPM_CLOSE
            : ShooterConstants.RPM_FAR;
    }

    /** RPM → ホイール周速 (m/s) に変換する。 */
    private static double rpmToMps(double rpm) {
        return rpm / 60.0 * Math.PI * ShooterConstants.FLYWHEEL_DIAMETER_M;
    }

    // ── DTO ───────────────────────────────────────────────────

    /**
     * ShotCalculator が返す射出パラメータ。
     *
     * @param hoodAngleDeg  Hood 目標角度 (deg)
     * @param shooterRpm    Shooter 目標 RPM
     * @param headingToHubRad HUB への方位角 (rad、フィールド基準)
     * @param distanceM     HUB までの水平距離 (m)
     */
    public record ShotParameters(
        double hoodAngleDeg,
        double shooterRpm,
        double headingToHubRad,
        double distanceM
    ) {}
}
