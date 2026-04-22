package frc.robot.subsystems.extension;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ExtensionConstants;

/**
 * アーム・クライマー等の伸展機構を制御するサブシステム。
 * CTRE KrakenX60 (TalonFX / Phoenix 6) + ThroughBoreEncoder (DIO 0) を使用する。
 * 起動時に ThroughBoreEncoder の絶対値で TalonFX 内部エンコーダをシードする。
 */
public class ExtensionSubsystem extends SubsystemBase {

    private final TalonFX          motor;
    private final DutyCycleEncoder absoluteEncoder;
    private final PositionVoltage  positionRequest = new PositionVoltage(0).withSlot(0);

    private double targetPosition = 0.0;

    public ExtensionSubsystem() {
        motor           = new TalonFX(ExtensionConstants.MOTOR_ID);
        absoluteEncoder = new DutyCycleEncoder(ExtensionConstants.THROUGH_BORE_DIO_PORT);

        TalonFXConfiguration cfg = new TalonFXConfiguration();
        cfg.Slot0.kP = ExtensionConstants.KP;
        cfg.Slot0.kI = ExtensionConstants.KI;
        cfg.Slot0.kD = ExtensionConstants.KD;
        cfg.SoftwareLimitSwitch.ForwardSoftLimitThreshold = ExtensionConstants.SOFT_LIMIT_FWD;
        cfg.SoftwareLimitSwitch.ReverseSoftLimitThreshold = ExtensionConstants.SOFT_LIMIT_REV;
        cfg.SoftwareLimitSwitch.ForwardSoftLimitEnable    = true;
        cfg.SoftwareLimitSwitch.ReverseSoftLimitEnable    = true;
        cfg.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        cfg.CurrentLimits.SupplyCurrentLimit       = 40;
        cfg.CurrentLimits.SupplyCurrentLimitEnable = true;

        motor.getConfigurator().apply(cfg);
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Extension/Position",       motor.getPosition().getValueAsDouble());
        SmartDashboard.putNumber("Extension/AbsEncoder",     absoluteEncoder.get());
        SmartDashboard.putNumber("Extension/Target",         targetPosition);
        SmartDashboard.putBoolean("Extension/AtTarget",      isAtTarget());
    }

    // ── コマンド向け API ──────────────────────────────────────

    public void setPosition(double rotations) {
        targetPosition = rotations;
        motor.setControl(positionRequest.withPosition(rotations));
    }

    public void extend() {
        setPosition(ExtensionConstants.EXTEND_POSITION);
    }

    public void retract() {
        setPosition(ExtensionConstants.RETRACT_POSITION);
    }

    public void stop() {
        motor.stopMotor();
    }

    /** ThroughBoreEncoder の絶対値を使って TalonFX 内部エンコーダをゼロリセットする。 */
    public void resetEncoderFromAbsolute() {
        motor.setPosition(absoluteEncoder.get());
    }

    public void resetEncoder() {
        motor.setPosition(0.0);
    }

    public boolean isAtTarget() {
        return Math.abs(motor.getPosition().getValueAsDouble() - targetPosition)
            < ExtensionConstants.POSITION_TOLERANCE;
    }

    public double getPosition() {
        return motor.getPosition().getValueAsDouble();
    }
}
