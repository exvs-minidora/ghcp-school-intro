package frc.robot.util;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;

/**
 * ロボットの移動慣性を補正した射出方位を計算するユーティリティクラス。
 *
 * <h3>補正の原理</h3>
 * ボールが空中にある間もロボットは移動し続けるが、ボールは射出後にロボットの
 * 横方向速度成分を引き継ぐ。そのため、ターゲットをロボット速度 × 飛行時間分
 * 「逆方向にずらした仮想ターゲット」に向けることで補正を実現する:
 *
 * <pre>
 *   compensatedTarget = naiveTarget - (vx * flightTime, vy * flightTime)
 * </pre>
 *
 * <p>停止中はオフセットがゼロになるため、通常の Aim と同一結果になる。
 */
public final class VelocityAimOffset {

    private VelocityAimOffset() {}

    /**
     * 慣性補正後のロボット目標方位 (rad) を返す。
     *
     * @param robotSpeeds   フィールド基準のロボット速度 (m/s)
     * @param flightTimeSec 推定ボール飛行時間 (s)。{@link ShotCalculator#estimateFlightTime} で取得。
     * @param robotPos      ロボットの現在位置 (Translation2d)
     * @param naiveTarget   補正前のターゲット位置（HUB 中央・Landing Zone 等）
     * @return 補正後の目標方位 (rad)
     */
    public static double compensate(
        ChassisSpeeds robotSpeeds,
        double        flightTimeSec,
        Translation2d robotPos,
        Translation2d naiveTarget
    ) {
        // ロボット速度 × 飛行時間分だけターゲットをオフセット（逆方向）
        double offsetX = robotSpeeds.vxMetersPerSecond * flightTimeSec;
        double offsetY = robotSpeeds.vyMetersPerSecond * flightTimeSec;

        Translation2d compensated = new Translation2d(
            naiveTarget.getX() - offsetX,
            naiveTarget.getY() - offsetY
        );

        return Math.atan2(
            compensated.getY() - robotPos.getY(),
            compensated.getX() - robotPos.getX()
        );
    }
}
