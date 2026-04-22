package frc.robot.subsystems.intake;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeConstants;

/**
 * フィールドのノートを吸入するインテークサブシステム。
 * REV SPARK MAX を使用する。
 */
public class IntakeSubsystem extends SubsystemBase {

    private final CANSparkMax motor;

    public IntakeSubsystem() {
        motor = new CANSparkMax(IntakeConstants.MOTOR_ID, MotorType.kBrushless);
        motor.restoreFactoryDefaults();
        motor.setSmartCurrentLimit(40);
        motor.burnFlash();
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Intake/OutputVoltage",
            motor.getAppliedOutput() * motor.getBusVoltage());
    }

    // ── コマンド向け API ──────────────────────────────────────

    /** ノートを吸入する方向に回転する。 */
    public void intake() {
        motor.set(IntakeConstants.INTAKE_SPEED);
    }

    /** ノートを吐き出す方向に回転する。 */
    public void eject() {
        motor.set(IntakeConstants.EJECT_SPEED);
    }

    /** モータを停止する。 */
    public void stop() {
        motor.stopMotor();
    }
}
