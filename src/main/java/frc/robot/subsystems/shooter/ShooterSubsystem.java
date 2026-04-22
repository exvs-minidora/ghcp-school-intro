package frc.robot.subsystems.shooter;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ShooterConstants;

/**
 * 射出ホイールを 2 モータで制御するサブシステム。
 * CTRE KrakenX60 (TalonFX / Phoenix 6) を使用する。
 * 速度単位は RPS (rotations per second) で統一し、外部 RPM を変換して渡す。
 * モータ配置: leftMotor / rightMotor (水平横並び、ボール下方から射出)
 */
public class ShooterSubsystem extends SubsystemBase {

    private final TalonFX leftMotor;
    private final TalonFX rightMotor;

    private final VelocityVoltage leftRequest  = new VelocityVoltage(0).withSlot(0);
    private final VelocityVoltage rightRequest = new VelocityVoltage(0).withSlot(0);

    private double targetLeftRpm  = 0.0;
    private double targetRightRpm = 0.0;

    // Flywheel debounce — RPM 範囲内を一定時間維持して初めて READY とする
    private final Timer debounceTimer   = new Timer();
    private boolean     debounceRunning = false;
    private boolean     flywheelReady   = false;

    public ShooterSubsystem() {
        leftMotor  = new TalonFX(ShooterConstants.LEFT_MOTOR_ID);
        rightMotor = new TalonFX(ShooterConstants.RIGHT_MOTOR_ID);

        TalonFXConfiguration cfg = new TalonFXConfiguration();
        cfg.Slot0.kP = ShooterConstants.KP;
        cfg.Slot0.kI = ShooterConstants.KI;
        cfg.Slot0.kD = ShooterConstants.KD;
        cfg.Slot0.kV = ShooterConstants.KV;
        cfg.CurrentLimits.SupplyCurrentLimit       = 60;
        cfg.CurrentLimits.SupplyCurrentLimitEnable = true;

        leftMotor.getConfigurator().apply(cfg);
        rightMotor.getConfigurator().apply(cfg);
    }

    @Override
    public void periodic() {
        double leftRpm  = leftMotor.getVelocity().getValueAsDouble()  * 60.0;
        double rightRpm = rightMotor.getVelocity().getValueAsDouble() * 60.0;

        boolean withinTol = targetLeftRpm > 0.0
            && Math.abs(leftRpm  - targetLeftRpm)  < ShooterConstants.RPM_TOLERANCE
            && Math.abs(rightRpm - targetRightRpm) < ShooterConstants.RPM_TOLERANCE;

        if (withinTol) {
            if (!debounceRunning) {
                debounceTimer.restart();
                debounceRunning = true;
            }
            flywheelReady = debounceTimer.hasElapsed(ShooterConstants.FLYWHEEL_DEBOUNCE_S);
        } else {
            debounceTimer.stop();
            debounceRunning = false;
            flywheelReady   = false;
        }

        SmartDashboard.putNumber("Shooter/LeftRPM",  leftRpm);
        SmartDashboard.putNumber("Shooter/RightRPM", rightRpm);
        SmartDashboard.putNumber("Shooter/TargetRPM", targetLeftRpm);
        SmartDashboard.putBoolean("Shooter/AtSpeed",  flywheelReady);
    }

    // ── コマンド向け API ──────────────────────────────────────

    public void setVelocity(double leftRpm, double rightRpm) {
        targetLeftRpm  = leftRpm;
        targetRightRpm = rightRpm;
        leftMotor.setControl(leftRequest.withVelocity(leftRpm  / 60.0));
        rightMotor.setControl(rightRequest.withVelocity(rightRpm / 60.0));
    }

    public void setVelocity(double rpm) {
        setVelocity(rpm, rpm);
    }

    public void spinUp() {
        setVelocity(ShooterConstants.DEFAULT_LEFT_RPM, ShooterConstants.DEFAULT_RIGHT_RPM);
    }

    public void stop() {
        targetLeftRpm = targetRightRpm = 0.0;
        flywheelReady   = false;
        debounceRunning = false;
        debounceTimer.stop();
        leftMotor.stopMotor();
        rightMotor.stopMotor();
    }

    public boolean isAtVelocity() {
        return flywheelReady;
    }
}
