package frc.robot.commands.shooter;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.CarryConstants;
import frc.robot.Constants.DrivetrainPhysics;
import frc.robot.Constants.FieldConstants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.Constants.SpindexerConstants;
import frc.robot.localization.PoseEstimatorSubsystem;
import frc.robot.subsystems.drivetrain.DrivebaseSubsystem;
import frc.robot.subsystems.feeder.FeederSubsystem;
import frc.robot.subsystems.hood.HoodSubsystem;
import frc.robot.subsystems.shooter.ShooterSubsystem;
import frc.robot.subsystems.spindexer.SpindexerSubsystem;
import frc.robot.util.ShotCalculator;
import frc.robot.util.VelocityAimOffset;

/**
 * NEUTRAL ZONE から自陣 Landing Zone へボールを「パス射出」するコマンド。
 *
 * <h3>制御フロー</h3>
 * <ul>
 *   <li>Hood を鋭角固定 ({@link CarryConstants#CARRY_HOOD_ANGLE_DEG}) に設定する</li>
 *   <li>RPM は {@link ShooterConstants#RPM_CARRY} に固定する</li>
 *   <li>Alliance を参照して Landing Zone 方向へドライブを向ける</li>
 *   <li>慣性補正 ({@link VelocityAimOffset}) でロボット速度を加味する</li>
 *   <li>Flywheel Ready + Hood Ready + Heading 収束の 3 条件成立で Feeder を起動する</li>
 * </ul>
 *
 * <p>Landing Zone 座標は {@link FieldConstants#LANDING_ZONE_BLUE} /
 * {@link FieldConstants#LANDING_ZONE_RED} を使用する。
 * 座標は公式フィールド図面確認後に更新すること。
 */
public class CarryShootCommand extends Command {

    private static final double HEADING_KP = 4.0;
    private static final double HEADING_KI = 0.0;
    private static final double HEADING_KD = 0.1;

    private final DrivebaseSubsystem     drivetrain;
    private final PoseEstimatorSubsystem poseEstimator;
    private final HoodSubsystem          hood;
    private final ShooterSubsystem       shooter;
    private final FeederSubsystem        feeder;
    private final SpindexerSubsystem     spindexer;

    private final PIDController headingPID;

    private final Timer spindexerTimer = new Timer();
    private boolean     feederStarted  = false;

    public CarryShootCommand(
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
        headingPID.setTolerance(Math.toRadians(CarryConstants.HEADING_TOLERANCE_DEG));

        addRequirements(drivetrain, hood, shooter, feeder, spindexer);
    }

    @Override
    public void initialize() {
        headingPID.reset();
        hood.setAngleDeg(CarryConstants.CARRY_HOOD_ANGLE_DEG);
        shooter.setVelocity(ShooterConstants.RPM_CARRY);
        feeder.stop();
        spindexer.stop();
        feederStarted = false;
        spindexerTimer.stop();
        spindexerTimer.reset();
    }

    @Override
    public void execute() {
        var pose   = poseEstimator.getPose();
        Translation2d landingZone = isRedAlliance()
            ? FieldConstants.LANDING_ZONE_RED
            : FieldConstants.LANDING_ZONE_BLUE;

        // 慣性補正: キャリーショットの推定飛行時間を利用する
        double flightTime = ShotCalculator.estimateFlightTime(
            pose.getTranslation().getDistance(landingZone),
            CarryConstants.CARRY_HOOD_ANGLE_DEG,
            ShooterConstants.RPM_CARRY
        );
        double correctedHeading = VelocityAimOffset.compensate(
            drivetrain.getRobotSpeedsMPS(), flightTime,
            pose.getTranslation(), landingZone
        );

        // Heading PID
        double rotOutput = headingPID.calculate(
            pose.getRotation().getRadians(), correctedHeading);
        rotOutput = MathUtil.clamp(rotOutput,
            -DrivetrainPhysics.MAX_ANGULAR, DrivetrainPhysics.MAX_ANGULAR);
        drivetrain.driveRobotRelative(new ChassisSpeeds(0.0, 0.0, rotOutput));

        // 3 条件ゲート
        boolean flywheelOk = shooter.isAtVelocity();
        boolean hoodOk     = hood.isAtTarget();
        boolean headingOk  = headingPID.atSetpoint();

        if (flywheelOk && hoodOk && headingOk) {
            feeder.feed();
            if (!feederStarted) {
                feederStarted = true;
                spindexerTimer.restart();
            }
            if (spindexerTimer.hasElapsed(SpindexerConstants.SPINDEXER_DELAY_S)) {
                spindexer.spin();
            }
        } else {
            feeder.stop();
            spindexer.stop();
            feederStarted = false;
            spindexerTimer.stop();
            spindexerTimer.reset();
        }

        // テレメトリ
        SmartDashboard.putBoolean("Carry/FlywheelReady", flywheelOk);
        SmartDashboard.putBoolean("Carry/HoodReady",     hoodOk);
        SmartDashboard.putBoolean("Carry/HeadingReady",  headingOk);
        SmartDashboard.putNumber("Carry/FlightTimeSec",  flightTime);
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
        return false; // ボタン保持型
    }

    private boolean isRedAlliance() {
        return DriverStation.getAlliance()
            .filter(a -> a == DriverStation.Alliance.Red)
            .isPresent();
    }
}
