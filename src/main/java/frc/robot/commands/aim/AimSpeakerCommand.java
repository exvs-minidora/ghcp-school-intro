package frc.robot.commands.aim;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.DrivetrainPhysics;
import frc.robot.Constants.FieldConstants;
import frc.robot.localization.PoseEstimatorSubsystem;
import frc.robot.subsystems.drivetrain.DrivebaseSubsystem;

/**
 * 現在 Pose から Speaker へロボットを向ける Aim コマンド。
 *
 * <p>ヘディングのみを PID 制御し、並進入力はゼロにする。
 * Alliance 設定を参照してターゲット座標を切り替える。
 */
public class AimSpeakerCommand extends Command {

    private static final double KP  = 4.0;
    private static final double KI  = 0.0;
    private static final double KD  = 0.1;

    private static final double HEADING_TOLERANCE_DEG = 2.0;

    private final DrivebaseSubsystem     drivetrain;
    private final PoseEstimatorSubsystem poseEstimator;
    private final PIDController          headingPID;

    public AimSpeakerCommand(DrivebaseSubsystem drivetrain,
                              PoseEstimatorSubsystem poseEstimator) {
        this.drivetrain    = drivetrain;
        this.poseEstimator = poseEstimator;

        headingPID = new PIDController(KP, KI, KD);
        headingPID.enableContinuousInput(-Math.PI, Math.PI);
        headingPID.setTolerance(Math.toRadians(HEADING_TOLERANCE_DEG));

        addRequirements(drivetrain);
    }

    @Override
    public void initialize() {
        headingPID.reset();
    }

    @Override
    public void execute() {
        Translation2d target = isRedAlliance()
            ? FieldConstants.SPEAKER_RED
            : FieldConstants.SPEAKER_BLUE;

        Translation2d robotPos = poseEstimator.getPose().getTranslation();
        double angleToTarget =
            Math.atan2(target.getY() - robotPos.getY(),
                       target.getX() - robotPos.getX());

        double currentYaw = poseEstimator.getPose().getRotation().getRadians();
        double rotOutput  = headingPID.calculate(currentYaw, angleToTarget);
        rotOutput = MathUtil.clamp(rotOutput,
            -DrivetrainPhysics.MAX_ANGULAR, DrivetrainPhysics.MAX_ANGULAR);

        drivetrain.driveRobotRelative(
            new ChassisSpeeds(0.0, 0.0, rotOutput));

        SmartDashboard.putNumber("Aim/AngleToTarget", Math.toDegrees(angleToTarget));
        SmartDashboard.putBoolean("Aim/OnTarget", headingPID.atSetpoint());
    }

    @Override
    public void end(boolean interrupted) {
        drivetrain.driveRobotRelative(new ChassisSpeeds());
    }

    @Override
    public boolean isFinished() {
        return false; // トリガー保持中は継続する
    }

    private boolean isRedAlliance() {
        return DriverStation.getAlliance()
            .filter(a -> a == DriverStation.Alliance.Red)
            .isPresent();
    }
}
