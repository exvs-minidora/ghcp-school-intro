package frc.robot.commands.drive;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.DrivetrainPhysics;
import frc.robot.Constants.OperatorConstants;
import frc.robot.localization.PoseEstimatorSubsystem;
import frc.robot.subsystems.drivetrain.DrivebaseSubsystem;

import java.util.function.DoubleSupplier;

/**
 * Teleop 走行コマンド。
 * ドライバーのジョイスティック入力をフィールド基準の ChassisSpeeds に変換する。
 *
 * <p>Red Alliance 時は入力を 180° 反転してフィールド基準を保つ。
 */
public class TeleopSwerveCommand extends Command {

    private final DrivebaseSubsystem     drivetrain;
    private final PoseEstimatorSubsystem poseEstimator;
    private final DoubleSupplier         xSupplier;   // 前進 (+前)
    private final DoubleSupplier         ySupplier;   // 左右 (+左)
    private final DoubleSupplier         rotSupplier; // 回転 (+反時計回り)

    public TeleopSwerveCommand(
        DrivebaseSubsystem     drivetrain,
        PoseEstimatorSubsystem poseEstimator,
        DoubleSupplier         xSupplier,
        DoubleSupplier         ySupplier,
        DoubleSupplier         rotSupplier
    ) {
        this.drivetrain    = drivetrain;
        this.poseEstimator = poseEstimator;
        this.xSupplier     = xSupplier;
        this.ySupplier     = ySupplier;
        this.rotSupplier   = rotSupplier;
        addRequirements(drivetrain);
    }

    @Override
    public void execute() {
        double deadband = OperatorConstants.JOYSTICK_DEADBAND;

        double vx  = MathUtil.applyDeadband(xSupplier.getAsDouble(),   deadband);
        double vy  = MathUtil.applyDeadband(ySupplier.getAsDouble(),   deadband);
        double rot = MathUtil.applyDeadband(rotSupplier.getAsDouble(), deadband);

        // Red Alliance では X/Y を反転してフィールド基準を統一する
        boolean isRed = DriverStation.getAlliance()
            .filter(a -> a == DriverStation.Alliance.Red).isPresent();
        if (isRed) {
            vx  = -vx;
            vy  = -vy;
        }

        vx  *= DrivetrainPhysics.MAX_SPEED;
        vy  *= DrivetrainPhysics.MAX_SPEED;
        rot *= DrivetrainPhysics.MAX_ANGULAR;

        drivetrain.driveFieldRelative(
            ChassisSpeeds.fromFieldRelativeSpeeds(
                vx, vy, rot, poseEstimator.getPose().getRotation()));
    }

    @Override
    public void end(boolean interrupted) {
        drivetrain.driveRobotRelative(new ChassisSpeeds());
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
