package frc.robot.subsystems.feeder;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.FeederConstants;

/**
 * Shooter へノートを送り込むフィーダサブシステム。
 * REV SPARK MAX (CANSparkMax) を使用する。
 */
public class FeederSubsystem extends SubsystemBase {

    private final CANSparkMax motor;

    public FeederSubsystem() {
        motor = new CANSparkMax(FeederConstants.MOTOR_ID, MotorType.kBrushless);
        motor.restoreFactoryDefaults();
        motor.setSmartCurrentLimit(30);
        motor.burnFlash();
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Feeder/OutputVoltage",
            motor.getAppliedOutput() * motor.getBusVoltage());
    }

    // ── コマンド向け API ──────────────────────────────────────

    /** フィード方向に動かす。 */
    public void feed() {
        motor.set(FeederConstants.FEED_SPEED);
    }

    /** ノートを押し戻す (障害物対応)。 */
    public void eject() {
        motor.set(FeederConstants.EJECT_SPEED);
    }

    /** モータを停止する。 */
    public void stop() {
        motor.stopMotor();
    }
}
