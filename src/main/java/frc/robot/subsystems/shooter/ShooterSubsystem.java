package frc.robot.subsystems.shooter;

import com.revrobotics.CANSparkFlex;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
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
        SmartDashboard.putNumber("Shooter/TopRPM",    topEncoder.getVelocity());
        SmartDashboard.putNumber("Shooter/BottomRPM", bottomEncoder.getVelocity());
        SmartDashboard.putBoolean("Shooter/AtSpeed",  isAtVelocity());
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

    /** デフォルト RPM (Constants 参照) で射出準備する。 */
    public void spinUp() {
        setVelocity(ShooterConstants.DEFAULT_TOP_RPM, ShooterConstants.DEFAULT_BOTTOM_RPM);
    }

    /** モータを停止する。 */
    public void stop() {
        targetTopRpm = targetBottomRpm = 0.0;
        topMotor.stopMotor();
        bottomMotor.stopMotor();
    }

    /** 両ホイールが目標 RPM 内に収束しているか返す。 */
    public boolean isAtVelocity() {
        double topErr    = Math.abs(topEncoder.getVelocity()    - targetTopRpm);
        double bottomErr = Math.abs(bottomEncoder.getVelocity() - targetBottomRpm);
        return topErr < ShooterConstants.RPM_TOLERANCE
            && bottomErr < ShooterConstants.RPM_TOLERANCE;
    }
}
