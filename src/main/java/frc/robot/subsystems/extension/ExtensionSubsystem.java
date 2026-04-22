package frc.robot.subsystems.extension;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.SoftLimitDirection;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ExtensionConstants;

/**
 * アーム・クライマー等の伸展機構を制御するサブシステム。
 * REV SPARK MAX + 統合エンコーダで位置制御を行う。
 */
public class ExtensionSubsystem extends SubsystemBase {

    private final CANSparkMax motor;
    private final RelativeEncoder encoder;
    private final SparkPIDController pid;

    private double targetPosition = 0.0;

    public ExtensionSubsystem() {
        motor = new CANSparkMax(ExtensionConstants.MOTOR_ID, MotorType.kBrushless);
        motor.restoreFactoryDefaults();
        motor.setSmartCurrentLimit(40);

        // ソフトリミット設定
        motor.setSoftLimit(SoftLimitDirection.kForward,
            (float) ExtensionConstants.SOFT_LIMIT_FWD);
        motor.setSoftLimit(SoftLimitDirection.kReverse,
            (float) ExtensionConstants.SOFT_LIMIT_REV);
        motor.enableSoftLimit(SoftLimitDirection.kForward, true);
        motor.enableSoftLimit(SoftLimitDirection.kReverse, true);

        encoder = motor.getEncoder();
        pid     = motor.getPIDController();
        pid.setP(ExtensionConstants.KP);
        pid.setI(ExtensionConstants.KI);
        pid.setD(ExtensionConstants.KD);

        motor.burnFlash();
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Extension/Position",  encoder.getPosition());
        SmartDashboard.putNumber("Extension/Target",    targetPosition);
        SmartDashboard.putBoolean("Extension/AtTarget", isAtTarget());
    }

    // ── コマンド向け API ──────────────────────────────────────

    /** 指定した回転数位置へ動かす。 */
    public void setPosition(double rotations) {
        targetPosition = rotations;
        pid.setReference(rotations, ControlType.kPosition);
    }

    /** フル展開位置へ動かす。 */
    public void extend() {
        setPosition(ExtensionConstants.EXTEND_POSITION);
    }

    /** 格納位置へ動かす。 */
    public void retract() {
        setPosition(ExtensionConstants.RETRACT_POSITION);
    }

    /** モータを停止する。 */
    public void stop() {
        motor.stopMotor();
    }

    /** エンコーダ位置をゼロリセットする (格納位置を原点に)。 */
    public void resetEncoder() {
        encoder.setPosition(0.0);
    }

    /** 目標位置内に収束しているか返す。 */
    public boolean isAtTarget() {
        return Math.abs(encoder.getPosition() - targetPosition)
            < ExtensionConstants.POSITION_TOLERANCE;
    }

    /** 現在の位置 (回転数) を返す。 */
    public double getPosition() {
        return encoder.getPosition();
    }
}
