package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.OperatorConstants;
import frc.robot.autonomous.AutoFactory;
import frc.robot.commands.drive.LockWheelsCommand;
import frc.robot.commands.drive.TeleopSwerveCommand;
import frc.robot.commands.extension.ExtendCommand;
import frc.robot.commands.extension.RetractCommand;
import frc.robot.commands.feeder.FeedCommand;
import frc.robot.commands.intake.IntakeCommand;
import frc.robot.commands.shooter.AimAndShootCommand;
import frc.robot.commands.shooter.CarryShootCommand;
import frc.robot.commands.shooter.ShootCommand;
import frc.robot.localization.PoseEstimatorSubsystem;
import frc.robot.subsystems.drivetrain.DrivebaseSubsystem;
import frc.robot.subsystems.extension.ExtensionSubsystem;
import frc.robot.subsystems.feeder.FeederSubsystem;
import frc.robot.subsystems.hood.HoodSubsystem;
import frc.robot.subsystems.intake.IntakeSubsystem;
import frc.robot.subsystems.shooter.ShooterSubsystem;
import frc.robot.subsystems.spindexer.SpindexerSubsystem;
import frc.robot.vision.LimelightSubsystem;
import frc.robot.vision.VisionFusion;

/**
 * すべてのサブシステム・コマンド・ボタンバインドを統合するクラス。
 * コントローラは 1 台 (port 0) のみ使用する。
 *
 * <pre>
 * 左スティック XY   : 並進移動
 * 右スティック X    : 旋回
 * A  (hold)        : AimAndShoot — HUB 自動照準 + Hood + FlywheelReady ゲート付き射出
 * B  (hold)        : CarryShoot  — NEUTRAL ZONE キャリーショット
 * X  (hold)        : Intake
 * Y  (hold)        : ShootCommand — 手動フォールバック (Hood なし)
 * RB (hold → 離す) : Extension 展開 → 格納
 * LB (hold)        : Feeder 単独 (詰まり解除など)
 * Start            : ホイールロック (X フォーメーション)
 * POV 下           : ジャイロリセット
 * </pre>
 */
public class RobotContainer {

    // ── Subsystems ──────────────────────────────────────────────
    public final DrivebaseSubsystem   drivetrain  = new DrivebaseSubsystem();
    public final ShooterSubsystem     shooter     = new ShooterSubsystem();
    public final FeederSubsystem      feeder      = new FeederSubsystem();
    public final IntakeSubsystem      intake      = new IntakeSubsystem();
    public final ExtensionSubsystem   extension   = new ExtensionSubsystem();
    public final HoodSubsystem        hood        = new HoodSubsystem();
    public final SpindexerSubsystem   spindexer   = new SpindexerSubsystem();

    // ── Localization & Vision ───────────────────────────────────
    public final PoseEstimatorSubsystem poseEstimator =
        new PoseEstimatorSubsystem(drivetrain);
    public final LimelightSubsystem limelight =
        new LimelightSubsystem();
    public final VisionFusion visionFusion =
        new VisionFusion(poseEstimator, limelight);

    // ── Controller (1 台のみ, port 0) ──────────────────────────
    private final CommandXboxController controller =
        new CommandXboxController(OperatorConstants.DRIVER_CONTROLLER_PORT);

    // ── Autonomous ──────────────────────────────────────────────
    private final AutoFactory autoFactory;

    public RobotContainer() {
        autoFactory = new AutoFactory(this);
        configureDefaultCommands();
        configureButtonBindings();
    }

    private void configureDefaultCommands() {
        drivetrain.setDefaultCommand(
            new TeleopSwerveCommand(
                drivetrain,
                poseEstimator,
                () -> -controller.getLeftY(),
                () -> -controller.getLeftX(),
                () -> -controller.getRightX()
            )
        );
    }

    private void configureButtonBindings() {

        // A 長押し: HUB 自動照準 + Hood + FlywheelReady ゲート付き射出
        controller.a().whileTrue(new AimAndShootCommand(
            drivetrain, poseEstimator, hood, shooter, feeder, spindexer));

        // B 長押し: NEUTRAL ZONE キャリーショット
        controller.b().whileTrue(new CarryShootCommand(
            drivetrain, poseEstimator, hood, shooter, feeder, spindexer));

        // X 長押し: Intake
        controller.x().whileTrue(new IntakeCommand(intake));

        // Y 長押し: 手動 ShootCommand フォールバック (Flywheel / Hood 故障時)
        controller.y().whileTrue(new ShootCommand(shooter, feeder, spindexer));

        // RB 長押し → 離す: Extension 展開 → 格納
        controller.rightBumper()
            .onTrue(new ExtendCommand(extension))
            .onFalse(new RetractCommand(extension));

        // LB 長押し: Feeder 単独
        controller.leftBumper().whileTrue(new FeedCommand(feeder));

        // Start: ホイールロック (X フォーメーション)
        controller.start().whileTrue(new LockWheelsCommand(drivetrain));

        // POV 下: ジャイロリセット
        controller.povDown().onTrue(
            Commands.runOnce(poseEstimator::resetGyro, poseEstimator));
    }

    public Command getAutonomousCommand() {
        return autoFactory.getSelectedAuto();
    }
}
