package frc.robot;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.OperatorConstants;
import frc.robot.subsystems.extension.ExtensionSubsystem;
import frc.robot.subsystems.feeder.FeederSubsystem;
import frc.robot.subsystems.hood.HoodSubsystem;
import frc.robot.subsystems.intake.IntakeSubsystem;
import frc.robot.subsystems.shooter.ShooterSubsystem;
import frc.robot.subsystems.spindexer.SpindexerSubsystem;

/**
 * モーター動作確認専用コンテナ (second ブランチ)。
 * 足回りは使用しない。コントローラ 1 台で全 7 モーターを個別に正転/逆転確認する。
 *
 * <pre>
 * ── 正転 (ボタン押下中) ──────────────────────────────────────
 *   A          Shooter Left  +1500 RPM
 *   B          Shooter Right +1500 RPM
 *   X          Feeder        正転
 *   Y          Intake        正転
 *   RB         Hood          +20% duty
 *   RT (>0.5)  Extension     +20% duty
 *   Start      Spindexer     正転
 *
 * ── 逆転 (ボタン押下中) ──────────────────────────────────────
 *   POV 左     Shooter Left  -1500 RPM
 *   POV 右     Shooter Right -1500 RPM
 *   POV 下     Feeder        逆転
 *   POV 上     Intake        逆転
 *   LB         Hood          -20% duty
 *   LT (>0.5)  Extension     -20% duty
 *   Back       Spindexer     逆転
 * </pre>
 *
 * <h3>記録シート</h3>
 * <pre>
 * ShooterLeft  (CAN 20): A=正転□ POV左=逆転□  → inverted: [    ]
 * ShooterRight (CAN 21): B=正転□ POV右=逆転□  → inverted: [    ]
 * Feeder       (CAN 22): X=正転□ POV下=逆転□  → inverted: [    ]
 * Intake       (CAN 23): Y=正転□ POV上=逆転□  → inverted: [    ]
 * Hood         (CAN 25): RB=正転□ LB=逆転□    → inverted: [    ]
 * Extension    (CAN 24): RT=正転□ LT=逆転□    → inverted: [    ]
 * Spindexer    (CAN 26): Start=正転□ Back=逆転□ → inverted: [    ]
 * </pre>
 */
public class MotorTestRobotContainer {

    private final ShooterSubsystem   shooter   = new ShooterSubsystem();
    private final FeederSubsystem    feeder    = new FeederSubsystem();
    private final IntakeSubsystem    intake    = new IntakeSubsystem();
    private final HoodSubsystem      hood      = new HoodSubsystem();
    private final ExtensionSubsystem extension = new ExtensionSubsystem();
    private final SpindexerSubsystem spindexer = new SpindexerSubsystem();

    private final CommandXboxController ctrl =
        new CommandXboxController(OperatorConstants.DRIVER_CONTROLLER_PORT);

    public MotorTestRobotContainer() {
        configureBindings();
    }

    private void configureBindings() {

        // ── Shooter Left (CAN 20) ────────────────────────────────
        // A: +1500 RPM (正転)
        ctrl.a().whileTrue(Commands.startEnd(
            () -> shooter.setVelocity(1500.0, 0.0),
            shooter::stop,
            shooter));

        // POV 左: -1500 RPM (逆転)
        ctrl.povLeft().whileTrue(Commands.startEnd(
            () -> shooter.setVelocity(-1500.0, 0.0),
            shooter::stop,
            shooter));

        // ── Shooter Right (CAN 21) ───────────────────────────────
        // B: +1500 RPM (正転)
        ctrl.b().whileTrue(Commands.startEnd(
            () -> shooter.setVelocity(0.0, 1500.0),
            shooter::stop,
            shooter));

        // POV 右: -1500 RPM (逆転)
        ctrl.povRight().whileTrue(Commands.startEnd(
            () -> shooter.setVelocity(0.0, -1500.0),
            shooter::stop,
            shooter));

        // ── Feeder (CAN 22) ──────────────────────────────────────
        // X: 正転
        ctrl.x().whileTrue(Commands.startEnd(
            feeder::feed,
            feeder::stop,
            feeder));

        // POV 下: 逆転
        ctrl.povDown().whileTrue(Commands.startEnd(
            feeder::eject,
            feeder::stop,
            feeder));

        // ── Intake (CAN 23) ──────────────────────────────────────
        // Y: 正転
        ctrl.y().whileTrue(Commands.startEnd(
            intake::intake,
            intake::stop,
            intake));

        // POV 上: 逆転
        ctrl.povUp().whileTrue(Commands.startEnd(
            intake::eject,
            intake::stop,
            intake));

        // ── Hood (CAN 25) ────────────────────────────────────────
        // RB: +20% duty (正転)
        ctrl.rightBumper().whileTrue(Commands.startEnd(
            () -> hood.setOutput(0.20),
            hood::stop,
            hood));

        // LB: -20% duty (逆転)
        ctrl.leftBumper().whileTrue(Commands.startEnd(
            () -> hood.setOutput(-0.20),
            hood::stop,
            hood));

        // ── Extension (CAN 24) ───────────────────────────────────
        // RT (>0.5): +20% duty (正転)
        ctrl.rightTrigger(0.5).whileTrue(Commands.startEnd(
            () -> extension.setOutput(0.20),
            extension::stop,
            extension));

        // LT (>0.5): -20% duty (逆転)
        ctrl.leftTrigger(0.5).whileTrue(Commands.startEnd(
            () -> extension.setOutput(-0.20),
            extension::stop,
            extension));

        // ── Spindexer (CAN 26) ───────────────────────────────────
        // Start: 正転
        ctrl.start().whileTrue(Commands.startEnd(
            spindexer::spin,
            spindexer::stop,
            spindexer));

        // Back: 逆転
        ctrl.back().whileTrue(Commands.startEnd(
            spindexer::reverse,
            spindexer::stop,
            spindexer));
    }
}
