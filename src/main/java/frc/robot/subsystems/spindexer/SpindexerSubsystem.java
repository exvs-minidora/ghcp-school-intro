package frc.robot.subsystems.spindexer;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.SpindexerConstants;

/**
 * ボールをシューターへ送り込む回転式インデクサ (Spindexer) のサブシステム。
 * CTRE KrakenX60 (TalonFX / Phoenix 6) を Duty-Cycle で制御する。
 *
 * <p>射出時はコマンド側のタイマーで Feeder 起動から {@link SpindexerConstants#SPINDEXER_DELAY_S}
 * 秒後に {@link #spin()} を呼ぶ段階起動パターンを想定している。
 */
public class SpindexerSubsystem extends SubsystemBase {

    private final TalonFX    motor;
    private final DutyCycleOut spinRequest    = new DutyCycleOut(0);
    private final DutyCycleOut reverseRequest = new DutyCycleOut(0);

    public SpindexerSubsystem() {
        motor = new TalonFX(SpindexerConstants.MOTOR_ID);

        TalonFXConfiguration cfg = new TalonFXConfiguration();
        cfg.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        cfg.CurrentLimits.SupplyCurrentLimit       = 30;
        cfg.CurrentLimits.SupplyCurrentLimitEnable = true;

        motor.getConfigurator().apply(cfg);
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Spindexer/DutyCycle",
            motor.getDutyCycle().getValueAsDouble());
    }

    // ── コマンド向け API ──────────────────────────────────────

    /** 正転 (射出方向) で Spindexer を回す。 */
    public void spin() {
        motor.setControl(spinRequest.withOutput(SpindexerConstants.SPIN_SPEED));
    }

    /** 逆転 (詰まり解除) で Spindexer を回す。 */
    public void reverse() {
        motor.setControl(reverseRequest.withOutput(SpindexerConstants.REVERSE_SPEED));
    }

    /** Spindexer を停止する。 */
    public void stop() {
        motor.stopMotor();
    }
}
