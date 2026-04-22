package frc.robot.commands.shooter;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.DrivetrainPhysics;
import frc.robot.Constants.FieldConstants;
import frc.robot.Constants.SpindexerConstants;
import frc.robot.localization.PoseEstimatorSubsystem;
import frc.robot.subsystems.drivetrain.DrivebaseSubsystem;
import frc.robot.subsystems.feeder.FeederSubsystem;
import frc.robot.subsystems.hood.HoodSubsystem;
import frc.robot.subsystems.shooter.ShooterSubsystem;
import frc.robot.subsystems.spindexer.SpindexerSubsystem;
import frc.robot.util.ShotCalculator;
import frc.robot.util.ShotCalculator.ShotParameters;
import frc.robot.util.VelocityAimOffset;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;

/**
 * HUB に向けた自動 Aim + Hood 角度調整 + FlywheelReady ゲート付き射出コマンド。
 * Shooter 収束後 Feeder を起動し、{@link SpindexerConstants#SPINDEXER_DELAY_S} 秒後に
 * Spindexer を段階起動する。
 */
public class AimAndShootCommand extends Command {

    private static final double HEADING_KP  = 4.0;
    private static final double HEADING_KI  = 0.0;
    private static final double HEADING_KD  = 0.1;
    private static final double HEADING_TOL = Math.toRadians(2.0);

    private final DrivebaseSubsystem     drivetrain;
    private final PoseEstimatorSubsystem poseEstimator;
    private final HoodSubsystem          hood;
    private final ShooterSubsystem       shooter;
    private final FeederSubsystem        feeder;
    private final SpindexerSubsystem     spindexer;

    private final PIDController headingPID;

    // Spindexer 段階起動タイマー — Feeder 起動開始からの経過時間を計測する
    private final Timer spindexerTimer  = new Timer();
    private boolean     feederStarted   = false;

    public AimAndShootCommand(
        DrivebaseSubsystem     drivetrain,
        PoseEstimatorSubsystem poseEstimator,
        HoodSubsystem          hood,
        ShooterSubsystem       shooter,
        FeederSubsystem        feeder,
        SpindexerSubsystem     spindexer
    ) {
        this.drivetrain    = drivetrain;
        this.poseEstimator = poseEstimator;
        this.hood          = hood;
        this.shooter       = shooter;
        this.feeder        = feeder;
        this.spindexer     = spindexer;

        headingPID = new PIDController(HEADING_KP, HEADING_KI, HEADING_KD);
        headingPID.enableContinuousInput(-Math.PI, Math.PI);
        headingPID.setTolerance(HEADING_TOL);

        addRequirements(drivetrain, hood, shooter, feeder, spindexer);
    }

    @Override
    public void initialize() {
        headingPID.reset();
        feeder.stop();
        spindexer.stop();
        feederStarted = false;
        spindexerTimer.stop();
        spindexerTimer.reset();
    }

    @Override
    public void execute() {
        var pose = poseEstimator.getPose();

        ShotParameters params = ShotCalculator.calculate(pose);

        double flightTime = ShotCalculator.estimateFlightTime(
            params.distanceM(), params.hoodAngleDeg(), params.shooterRpm());
        Translation2d hub = FieldConstants.HUB_CENTER;
        double correctedHeading = VelocityAimOffset.compensate(
            drivetrain.getRobotSpeedsMPS(), flightTime, pose.getTranslation(), hub);

        hood.setAngleDeg(params.hoodAngleDeg());
        shooter.setVelocity(params.shooterRpm());

        double currentYaw = pose.getRotation().getRadians();
        double rotOutput  = headingPID.calculate(currentYaw, correctedHeading);
        rotOutput = MathUtil.clamp(rotOutput,
            -DrivetrainPhysics.MAX_ANGULAR, DrivetrainPhysics.MAX_ANGULAR);
        drivetrain.driveRobotRelative(new ChassisSpeeds(0.0, 0.0, rotOutput));

        boolean flywheelOk = shooter.isAtVelocity();
        boolean hoodOk     = hood.isAtTarget();
        boolean headingOk  = headingPID.atSetpoint();
        boolean allReady   = flywheelOk && hoodOk && headingOk;

        if (allReady) {
            // STATE 2: Feeder 起動
            feeder.feed();
            if (!feederStarted) {
                feederStarted = true;
                spindexerTimer.restart();
            }
            // STATE 3: SPINDEXER_DELAY 経過後 Spindexer 起動
            if (spindexerTimer.hasElapsed(SpindexerConstants.SPINDEXER_DELAY_S)) {
                spindexer.spin();
            }
        } else {
            // STATE 1: 収束待ち
            feeder.stop();
            spindexer.stop();
            feederStarted = false;
            spindexerTimer.stop();
            spindexerTimer.reset();
        }

        SmartDashboard.putBoolean("AimShoot/FlywheelReady", flywheelOk);
        SmartDashboard.putBoolean("AimShoot/HoodReady",     hoodOk);
        SmartDashboard.putBoolean("AimShoot/HeadingReady",  headingOk);
        SmartDashboard.putNumber("AimShoot/DistanceM",      params.distanceM());
        SmartDashboard.putNumber("AimShoot/HoodAngle",      params.hoodAngleDeg());
        SmartDashboard.putNumber("AimShoot/TargetRPM",      params.shooterRpm());
        SmartDashboard.putNumber("AimShoot/FlightTimeSec",  flightTime);
    }

    @Override
    public void end(boolean interrupted) {
        feeder.stop();
        spindexer.stop();
        shooter.stop();
        hood.stow();
        drivetrain.driveRobotRelative(new ChassisSpeeds());
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
