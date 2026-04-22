package frc.robot.subsystems.shooter;

import com.revrobotics.CANSparkFlex;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ShooterConstants;

/**
 * 射出ホイールを 2 モータで制御するサブシステム。
 * REV SPARK Flex (CANSparkFlex) を使用する。
 */
public class ShooterSubsystem extends SubsystemBase {

    private final CANSparkFlex topMotor;
    private final CANSparkFlex bottomMotor;

    private final SparkPIDController topPID;
    private final SparkPIDController bottomPID;

    private final RelativeEncoder topEncoder;
    private final RelativeEncoder bottomEncoder;

    private double targetTopRpm    = 0.0;
    private double targetBottomRpm = 0.0;

    // Flywheel debounce — RPM 範囲内を一定時間維持して初めて READY とする
    private final Timer debounceTimer   = new Timer();
    private boolean     debounceRunning = false;
    private boolean     flywheelReady   = false;

    public ShooterSubsystem() {
        topMotor    = new CANSparkFlex(ShooterConstants.TOP_MOTOR_ID,    MotorType.kBrushless);
        bottomMotor = new CANSparkFlex(ShooterConstants.BOTTOM_MOTOR_ID, MotorType.kBrushless);

        topMotor.restoreFactoryDefaults();
        bottomMotor.restoreFactoryDefaults();

        // 電流制限 (A)
        topMotor.setSmartCurrentLimit(60);
        bottomMotor.setSmartCurrentLimit(60);

        topEncoder    = topMotor.getEncoder();
        bottomEncoder = bottomMotor.getEncoder();

        topPID    = topMotor.getPIDController();
        bottomPID = bottomMotor.getPIDController();

        for (SparkPIDController pid : new SparkPIDController[]{topPID, bottomPID}) {
            pid.setP(ShooterConstants.KP);
            pid.setI(ShooterConstants.KI);
            pid.setD(ShooterConstants.KD);
            pid.setFF(ShooterConstants.KV);
        }

        topMotor.burnFlash();
        bottomMotor.burnFlash();
    }

    @Override
    public void periodic() {
        double topRpm    = topEncoder.getVelocity();
        double bottomRpm = bottomEncoder.getVelocity();
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

    /**
     * 目標 RPM を設定して速度制御を開始する。
     *
     * @param topRpm    上ホイール目標 RPM
     * @param bottomRpm 下ホイール目標 RPM
     */
    public void setVelocity(double topRpm, double bottomRpm) {
        targetTopRpm    = topRpm;
        targetBottomRpm = bottomRpm;
        topPID.setReference(topRpm,    ControlType.kVelocity);
        bottomPID.setReference(bottomRpm, ControlType.kVelocity);
    }

    /**
     * 両ホイールに同一 RPM を設定する（上下同速の場合に使用）。
     *
     * @param rpm ボールの両ホイールの目標 RPM
     */
    public void setVelocity(double rpm) {
        setVelocity(rpm, rpm);
    }

    /** デフォルト RPM (Constants 参照) で射出準備する。 */
    public void spinUp() {
        setVelocity(ShooterConstants.DEFAULT_TOP_RPM, ShooterConstants.DEFAULT_BOTTOM_RPM);
    }

    /** モータを停止し、debounce ステートをリセットする。 */
    public void stop() {
        targetTopRpm = targetBottomRpm = 0.0;
        flywheelReady   = false;
        debounceRunning = false;
        debounceTimer.stop();
        topMotor.stopMotor();
        bottomMotor.stopMotor();
    }

    /**
     * 両ホイールが目標 RPM 内に {@link ShooterConstants#FLYWHEEL_DEBOUNCE_S} 以上維持しているか返す。
     * 目標 RPM が 0 のときは常に false を返す（未セット時の誤発火防止）。
     */
    public boolean isAtVelocity() {
        return flywheelReady;
    }
}
