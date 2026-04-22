package frc.robot.subsystems.hood;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.HoodConstants;

/**
 * 射出角度を調整する Hood サブシステム。
 * CTRE KrakenX60 (TalonFX / Phoenix 6) を使用する。
 * ギア比 10:1 で内部エンコーダ (回転数) → 角度 (deg) を換算する。
 */
public class HoodSubsystem extends SubsystemBase {

    private final TalonFX motor;
    private final PositionVoltage positionRequest = new PositionVoltage(0).withSlot(0);

    private double targetAngleDeg = HoodConstants.STOW_ANGLE_DEG;

    public HoodSubsystem() {
        motor = new TalonFX(HoodConstants.MOTOR_ID);

        TalonFXConfiguration cfg = new TalonFXConfiguration();
        cfg.Slot0.kP = HoodConstants.KP;
        cfg.Slot0.kI = HoodConstants.KI;
        cfg.Slot0.kD = HoodConstants.KD;
        cfg.SoftwareLimitSwitch.ForwardSoftLimitThreshold = HoodConstants.SOFT_LIMIT_FWD;
        cfg.SoftwareLimitSwitch.ReverseSoftLimitThreshold = HoodConstants.SOFT_LIMIT_REV;
        cfg.SoftwareLimitSwitch.ForwardSoftLimitEnable    = true;
        cfg.SoftwareLimitSwitch.ReverseSoftLimitEnable    = true;
        cfg.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        cfg.CurrentLimits.SupplyCurrentLimit       = 20;
        cfg.CurrentLimits.SupplyCurrentLimitEnable = true;

        motor.getConfigurator().apply(cfg);
        stow();
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Hood/AngleDeg",  getAngleDeg());
        SmartDashboard.putNumber("Hood/TargetDeg", targetAngleDeg);
        SmartDashboard.putBoolean("Hood/AtTarget", isAtTarget());
    }

    public void setAngleDeg(double angleDeg) {
        targetAngleDeg = MathUtil.clamp(angleDeg,
            HoodConstants.MIN_ANGLE_DEG, HoodConstants.MAX_ANGLE_DEG);
        motor.setControl(positionRequest.withPosition(degToRotations(targetAngleDeg)));
    }

    public void stow() {
        setAngleDeg(HoodConstants.STOW_ANGLE_DEG);
    }

    public void stop() {
        motor.stopMotor();
    }

    public double getAngleDeg() {
        return rotationsToDeg(motor.getPosition().getValueAsDouble());
    }

    public boolean isAtTarget() {
        return Math.abs(getAngleDeg() - targetAngleDeg) < HoodConstants.ANGLE_TOLERANCE_DEG;
    }

    public void resetEncoder() {
        motor.setPosition(degToRotations(HoodConstants.STOW_ANGLE_DEG));
    }

    private static double degToRotations(double deg) {
        return deg * HoodConstants.GEAR_RATIO / 360.0;
    }

    private static double rotationsToDeg(double rotations) {
        return rotations * 360.0 / HoodConstants.GEAR_RATIO;
    }
}
