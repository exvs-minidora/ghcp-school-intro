package frc.robot.commands.shooter;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.DrivetrainPhysics;
import frc.robot.Constants.FieldConstants;
import frc.robot.localization.PoseEstimatorSubsystem;
import frc.robot.subsystems.drivetrain.DrivebaseSubsystem;
import frc.robot.subsystems.feeder.FeederSubsystem;
import frc.robot.subsystems.hood.HoodSubsystem;
import frc.robot.subsystems.shooter.ShooterSubsystem;
import frc.robot.util.ShotCalculator;
import frc.robot.util.ShotCalculator.ShotParameters;
import frc.robot.util.VelocityAimOffset;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;

/**
 * HUB に向けた自動 Aim + Hood 角度調整 + FlywheelReady ゲート付き射出コマンド。
 *
 * <h3>制御フロー</h3>
 * <pre>
 * SEEKING (毎周期):
 *   1. Pose を取得し ShotCalculator で最適パラメータを算出する
 *   2. VelocityAimOffset でロボット慣性を補正した目標方位を決める
 *   3. Hood と Shooter の目標値を更新する
 *   4. Heading PID でドライブを制御する
 *   5. Feeder は停止のまま
 *
 * READY (3 条件同時成立時):
 *   - shooter.isAtVelocity()  … Flywheel が debounce 付きで RPM 収束
 *   - hood.isAtTarget()       … Hood 角度が収束
 *   - headingPID.atSetpoint() … ロボット向きが収束
 *   → feeder.feed() を呼ぶ
 *
 * end():
 *   feeder, shooter, hood (stow) を停止する
 * </pre>
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

    private final PIDController headingPID;

    public AimAndShootCommand(
        DrivebaseSubsystem     drivetrain,
        PoseEstimatorSubsystem poseEstimator,
        HoodSubsystem          hood,
        ShooterSubsystem       shooter,
        FeederSubsystem        feeder
    ) {
        this.drivetrain    = drivetrain;
        this.poseEstimator = poseEstimator;
        this.hood          = hood;
        this.shooter       = shooter;
        this.feeder        = feeder;

        headingPID = new PIDController(HEADING_KP, HEADING_KI, HEADING_KD);
        headingPID.enableContinuousInput(-Math.PI, Math.PI);
        headingPID.setTolerance(HEADING_TOL);

        addRequirements(drivetrain, hood, shooter, feeder);
    }

    @Override
    public void initialize() {
        headingPID.reset();
        feeder.stop();
    }

    @Override
    public void execute() {
        var pose = poseEstimator.getPose();

        // ── Shot パラメータ算出 ───────────────────────────────
        ShotParameters params = ShotCalculator.calculate(pose);

        // 慣性補正: ロボット速度 × 飛行時間 分だけターゲットをずらす
        double flightTime = ShotCalculator.estimateFlightTime(
            params.distanceM(), params.hoodAngleDeg(), params.shooterRpm());
        Translation2d hub = FieldConstants.HUB_CENTER;
        double correctedHeading = VelocityAimOffset.compensate(
            drivetrain.getRobotSpeedsMPS(), flightTime, pose.getTranslation(), hub);

        // ── Hood & Shooter 目標値更新 ─────────────────────────
        hood.setAngleDeg(params.hoodAngleDeg());
        shooter.setVelocity(params.shooterRpm());

        // ── Heading PID ───────────────────────────────────────
        double currentYaw  = pose.getRotation().getRadians();
        double rotOutput   = headingPID.calculate(currentYaw, correctedHeading);
        rotOutput = MathUtil.clamp(rotOutput,
            -DrivetrainPhysics.MAX_ANGULAR, DrivetrainPhysics.MAX_ANGULAR);
        drivetrain.driveRobotRelative(new ChassisSpeeds(0.0, 0.0, rotOutput));

        // ── Feeder ゲート (3 条件同時成立) ───────────────────
        boolean flywheelOk = shooter.isAtVelocity();
        boolean hoodOk     = hood.isAtTarget();
        boolean headingOk  = headingPID.atSetpoint();

        if (flywheelOk && hoodOk && headingOk) {
            feeder.feed();
        } else {
            feeder.stop();
        }

        // ── テレメトリ ────────────────────────────────────────
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
        shooter.stop();
        hood.stow();
        drivetrain.driveRobotRelative(new ChassisSpeeds());
    }

    @Override
    public boolean isFinished() {
        return false; // ボタン保持型
    }
}
