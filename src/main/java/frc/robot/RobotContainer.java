package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
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
 * RoboRIO 上で動作するエントリポイントとして機能する。
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

    // ── Controllers ─────────────────────────────────────────────
    private final CommandXboxController driver =
        new CommandXboxController(OperatorConstants.DRIVER_CONTROLLER_PORT);
    private final CommandXboxController operator =
        new CommandXboxController(OperatorConstants.OPERATOR_CONTROLLER_PORT);

    // ── Autonomous ──────────────────────────────────────────────
    private final AutoFactory autoFactory;

    public RobotContainer() {
        // PathPlanner AutoBuilder を初期化し Named Commands を登録する
        autoFactory = new AutoFactory(this);

        // デフォルトコマンドとボタンバインドを設定する
        configureDefaultCommands();
        configureButtonBindings();
    }

    /** 各サブシステムのデフォルトコマンドを設定する */
    private void configureDefaultCommands() {
        drivetrain.setDefaultCommand(
            new TeleopSwerveCommand(
                drivetrain,
                poseEstimator,
                () -> -driver.getLeftY(),
                () -> -driver.getLeftX(),
                () -> -driver.getRightX()
            )
        );
    }

    /** ボタンバインドを設定する */
    private void configureButtonBindings() {
        // ── Driver ──────────────────────────────────────────────
        // A ボタン長押し: HUB 自動 Aim + Hood + FlywheelReady ゲート付き射出
        driver.a().whileTrue(new AimAndShootCommand(
            drivetrain, poseEstimator, hood, shooter, feeder, spindexer));

        // B ボタン長押し: NEUTRAL ZONE キャリーショット (自陣へパス射出)
        driver.b().whileTrue(new CarryShootCommand(
            drivetrain, poseEstimator, hood, shooter, feeder, spindexer));

        // Start ボタン: ホイールロック (X フォーメーション)
        driver.start().whileTrue(new LockWheelsCommand(drivetrain));

        // ── Operator ────────────────────────────────────────────
        // 右バンパー: Intake
        operator.rightBumper().whileTrue(new IntakeCommand(intake));

        // 左バンパー: Extension 展開 / 離したら格納
        operator.leftBumper()
            .onTrue(new ExtendCommand(extension))
            .onFalse(new RetractCommand(extension));

        // A ボタン: 手動 Shoot フォールバック (Hood なし / Flywheel 故障時用)
        operator.a().whileTrue(new ShootCommand(shooter, feeder, spindexer));

        // B ボタン: Feeder 単独
        operator.b().whileTrue(new FeedCommand(feeder));
    }

    /**
     * 選択された自律コマンドを返す。
     * PathPlanner Auto Chooser の選択値が返される。
     */
    public Command getAutonomousCommand() {
        return autoFactory.getSelectedAuto();
    }
}
