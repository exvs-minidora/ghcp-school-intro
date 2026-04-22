package frc.robot.autonomous;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.DrivetrainPhysics;
import frc.robot.RobotContainer;
import frc.robot.commands.intake.IntakeCommand;
import frc.robot.commands.shooter.ShootCommand;
import frc.robot.commands.extension.ExtendCommand;
import frc.robot.commands.extension.RetractCommand;

/**
 * PathPlanner {@link AutoBuilder} の初期化と Named Commands 登録、
 * Auto 選択 UI を管理するクラス。
 *
 * <p>コンストラクタ呼び出し前に {@link frc.robot.localization.PoseEstimatorSubsystem} が
 * 初期化されている必要がある。
 */
public class AutoFactory {

    private final SendableChooser<Command> autoChooser;

    public AutoFactory(RobotContainer container) {
        // ── Named Commands 登録 ─────────────────────────────────
        // PathPlanner のイベントマーカーから参照される名前と一致させること
        NamedCommands.registerCommand(
            "Intake",
            new IntakeCommand(container.intake).withTimeout(2.0)
        );
        NamedCommands.registerCommand(
            "Shoot",
            new ShootCommand(container.shooter, container.feeder).withTimeout(2.0)
        );
        NamedCommands.registerCommand(
            "Extend",
            new ExtendCommand(container.extension)
        );
        NamedCommands.registerCommand(
            "Retract",
            new RetractCommand(container.extension)
        );

        // ── AutoBuilder 初期化 ───────────────────────────────────
        AutoBuilder.configureHolonomic(
            container.poseEstimator::getPose,
            container.poseEstimator::resetPose,
            container.drivetrain::getRobotSpeedsMPS,
            container.drivetrain::driveRobotRelative,
            new HolonomicPathFollowerConfig(
                new PIDConstants(5.0, 0.0, 0.0),  // 並進 PID
                new PIDConstants(5.0, 0.0, 0.0),  // 回転 PID
                DrivetrainPhysics.MAX_SPEED,
                DrivetrainPhysics.DRIVE_RADIUS,
                new ReplanningConfig()
            ),
            // Red Alliance の場合はパスを鏡像反転する
            () -> DriverStation.getAlliance()
                .filter(a -> a == DriverStation.Alliance.Red)
                .isPresent(),
            container.drivetrain
        );

        // ── Auto Chooser ─────────────────────────────────────────
        // deploy/pathplanner/autos/ 以下の .auto ファイルを自動認識する
        autoChooser = AutoBuilder.buildAutoChooser("DefaultAuto");
        SmartDashboard.putData("Auto/Chooser", autoChooser);
    }

    /** SmartDashboard の Chooser で選択された自律コマンドを返す。 */
    public Command getSelectedAuto() {
        return autoChooser.getSelected();
    }
}
