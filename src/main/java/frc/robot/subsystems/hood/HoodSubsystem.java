package frc.robot.subsystems.hood;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.SoftLimitDirection;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.HoodConstants;

/**
 * 射出角度を調整する Hood サブシステム。
 *
 * <p>REV SPARK MAX + 統合エンコーダで回転数 → 角度 (deg) の位置制御を行う。
 * 可動範囲は {@link HoodConstants#MIN_ANGLE_DEG} 〜 {@link HoodConstants#MAX_ANGLE_DEG}。
 * ロボット起動時は格納位置 ({@link HoodConstants#STOW_ANGLE_DEG}) へ移動する。
 *
 * <p>エンコーダ 1 回転 = 360 / GEAR_RATIO 度 として換算する。
 */
public class HoodSubsystem extends SubsystemBase {

    private final CANSparkMax        motor;
    private final RelativeEncoder    encoder;
    private final SparkPIDController pid;

    private double targetAngleDeg = HoodConstants.STOW_ANGLE_DEG;

    public HoodSubsystem() {
        motor = new CANSparkMax(HoodConstants.MOTOR_ID, MotorType.kBrushless);
        motor.restoreFactoryDefaults();
        motor.setSmartCurrentLimit(20);

        // ソフトリミット設定
        motor.setSoftLimit(SoftLimitDirection.kForward, (float) HoodConstants.SOFT_LIMIT_FWD);
        motor.setSoftLimit(SoftLimitDirection.kReverse, (float) HoodConstants.SOFT_LIMIT_REV);
        motor.enableSoftLimit(SoftLimitDirection.kForward, true);
        motor.enableSoftLimit(SoftLimitDirection.kReverse, true);

        encoder = motor.getEncoder();
        pid     = motor.getPIDController();
        pid.setP(HoodConstants.KP);
        pid.setI(HoodConstants.KI);
        pid.setD(HoodConstants.KD);

        motor.burnFlash();

        // 起動時に格納位置へ
        stow();
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Hood/AngleDeg",   getAngleDeg());
        SmartDashboard.putNumber("Hood/TargetDeg",  targetAngleDeg);
        SmartDashboard.putBoolean("Hood/AtTarget",  isAtTarget());
    }

    // ── 公開 API ──────────────────────────────────────────────

    /**
     * Hood を指定角度 (deg) へ動かす。範囲外はクランプする。
     *
     * @param angleDeg 目標角度 (deg)
     */
    public void setAngleDeg(double angleDeg) {
        targetAngleDeg = MathUtil.clamp(angleDeg,
            HoodConstants.MIN_ANGLE_DEG, HoodConstants.MAX_ANGLE_DEG);
        pid.setReference(degToRotations(targetAngleDeg), ControlType.kPosition);
    }

    /** 格納位置 ({@link HoodConstants#STOW_ANGLE_DEG}) へ動かす。 */
    public void stow() {
        setAngleDeg(HoodConstants.STOW_ANGLE_DEG);
    }

    /** モータを停止する（位置保持を解除）。 */
    public void stop() {
        motor.stopMotor();
    }

    /** 現在の角度 (deg) を返す。 */
    public double getAngleDeg() {
        return rotationsToDeg(encoder.getPosition());
    }

    /** 目標角度内に収束しているか返す。 */
    public boolean isAtTarget() {
        return Math.abs(getAngleDeg() - targetAngleDeg) < HoodConstants.ANGLE_TOLERANCE_DEG;
    }

    /** 起動時にエンコーダをゼロリセットする（格納位置を原点とする場合に呼ぶ）。 */
    public void resetEncoder() {
        encoder.setPosition(degToRotations(HoodConstants.STOW_ANGLE_DEG));
    }

    // ── 変換ヘルパー ──────────────────────────────────────────

    private static double degToRotations(double deg) {
        return deg * HoodConstants.GEAR_RATIO / 360.0;
    }

    private static double rotationsToDeg(double rotations) {
        return rotations * 360.0 / HoodConstants.GEAR_RATIO;
    }
}
