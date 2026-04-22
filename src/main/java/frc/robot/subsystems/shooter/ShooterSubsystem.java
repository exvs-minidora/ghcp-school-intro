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
 */
public class ShooterSubsystem extends SubsystemBase {

    private final TalonFX topMotor;
    private final TalonFX bottomMotor;

    private final VelocityVoltage topRequest    = new VelocityVoltage(0).withSlot(0);
    private final VelocityVoltage bottomRequest = new VelocityVoltage(0).withSlot(0);

    private double targetTopRpm    = 0.0;
    private double targetBottomRpm = 0.0;

    // Flywheel debounce — RPM 範囲内を一定時間維持して初めて READY とする
    private final Timer debounceTimer   = new Timer();
    private boolean     debounceRunning = false;
    private boolean     flywheelReady   = false;

    public ShooterSubsystem() {
        topMotor    = new TalonFX(ShooterConstants.TOP_MOTOR_ID);
        bottomMotor = new TalonFX(ShooterConstants.BOTTOM_MOTOR_ID);

        TalonFXConfiguration cfg = new TalonFXConfiguration();
        cfg.Slot0.kP = ShooterConstants.KP;
        cfg.Slot0.kI = ShooterConstants.KI;
        cfg.Slot0.kD = ShooterConstants.KD;
        cfg.Slot0.kV = ShooterConstants.KV;
        cfg.CurrentLimits.SupplyCurrentLimit       = 60;
        cfg.CurrentLimits.SupplyCurrentLimitEnable = true;

        topMotor.getConfigurator().apply(cfg);
        bottomMotor.getConfigurator().apply(cfg);
    }

    @Override
    public void periodic() {
        double topRpm    = topMotor.getVelocity().getValueAsDouble()    * 60.0;
        double bottomRpm = bottomMotor.getVelocity().getValueAsDouble() * 60.0;

        boolean withinTol = targetTopRpm > 0.0
            && Math.abs(topRpm    - targetTopRpm)    < ShooterConstants.RPM_TOLERANCE
            && Math.abs(bottomRpm - targetBottomRpm) < ShooterConstants.RPM_TOLERANCE;

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

        SmartDashboard.putNumber("Shooter/TopRPM",    topRpm);
        SmartDashboard.putNumber("Shooter/BottomRPM", bottomRpm);
        SmartDashboard.putNumber("Shooter/TargetRPM", targetTopRpm);
        SmartDashboard.putBoolean("Shooter/AtSpeed",  flywheelReady);
    }

    // ── コマンド向け API ──────────────────────────────────────

    public void setVelocity(double topRpm, double bottomRpm) {
        targetTopRpm    = topRpm;
        targetBottomRpm = bottomRpm;
        topMotor.setControl(topRequest.withVelocity(topRpm / 60.0));
        bottomMotor.setControl(bottomRequest.withVelocity(bottomRpm / 60.0));
    }

    public void setVelocity(double rpm) {
        setVelocity(rpm, rpm);
    }

    public void spinUp() {
        setVelocity(ShooterConstants.DEFAULT_TOP_RPM, ShooterConstants.DEFAULT_BOTTOM_RPM);
    }

    public void stop() {
        targetTopRpm = targetBottomRpm = 0.0;
        flywheelReady   = false;
        debounceRunning = false;
        debounceTimer.stop();
        topMotor.stopMotor();
        bottomMotor.stopMotor();
    }

    public boolean isAtVelocity() {
        return flywheelReady;
    }
}
