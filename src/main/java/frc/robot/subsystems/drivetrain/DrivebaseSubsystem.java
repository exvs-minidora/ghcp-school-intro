package frc.robot.subsystems.drivetrain;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DrivetrainPhysics;
import swervelib.SwerveDrive;
import swervelib.parser.SwerveParser;
import swervelib.telemetry.SwerveDriveTelemetry;
import swervelib.telemetry.SwerveDriveTelemetry.TelemetryVerbosity;

import java.io.File;

/**
 * YAGSL を用いた Swerve DriveBase サブシステム。
 *
 * <p>deploy/swerve/ 以下の JSON 設定から SwerveParser が初期化を行う。
 * Teleop 駆動は {@link TeleopSwerveCommand} 経由で {@link #drive} を呼び出す。
 * PathPlanner 追従は {@link #driveRobotRelative} を使用する。
 */
public class DrivebaseSubsystem extends SubsystemBase {

    private final SwerveDrive swerveDrive;
    private final Field2d field2d = new Field2d();

    public DrivebaseSubsystem() {
        // コンストラクタでの例外は Robot の起動を止めるため即座に再スロー
        SwerveDriveTelemetry.verbosity = TelemetryVerbosity.HIGH;
        try {
            File swerveConfigDir = new File(
                DrivebaseSubsystem.class.getResource("/swerve").toURI());
            swerveDrive = new SwerveParser(swerveConfigDir)
                .createSwerveDrive(DrivetrainPhysics.MAX_SPEED);
        } catch (Exception e) {
            throw new RuntimeException("Swerve 設定の読み込みに失敗しました", e);
        }
        swerveDrive.setHeadingCorrection(false);
        SmartDashboard.putData("Field", field2d);
    }

    // ── 周期処理 ──────────────────────────────────────────────

    @Override
    public void periodic() {
        field2d.setRobotPose(swerveDrive.getPose());

        SmartDashboard.putNumber("Drive/GyroYaw",
            swerveDrive.getYaw().getDegrees());
        SmartDashboard.putNumber("Drive/PoseX",
            swerveDrive.getPose().getX());
        SmartDashboard.putNumber("Drive/PoseY",
            swerveDrive.getPose().getY());
    }

    // ── 駆動 API ───────────────────────────────────────────────

    /**
     * フィールド基準またはロボット基準で速度を指定して走行する。
     *
     * @param translation   フィールド基準の X/Y 速度 (m/s × {@link ChassisSpeeds})
     * @param rotation      回転角速度 (rad/s)
     * @param fieldRelative true のときフィールド基準
     * @param openLoop      true のとき速度フィードバックを無効にする
     */
    public void drive(ChassisSpeeds translation, double rotation,
                      boolean fieldRelative, boolean openLoop) {
        swerveDrive.drive(translation, fieldRelative, openLoop);
    }

    /**
     * フィールド基準 ChassisSpeeds で直接指定する (Teleop 用ユーティリティ)。
     *
     * @param speeds フィールド基準の速度
     */
    public void driveFieldRelative(ChassisSpeeds speeds) {
        swerveDrive.driveFieldOriented(speeds);
    }

    /**
     * ロボット基準 ChassisSpeeds で走行する (PathPlanner AutoBuilder 向け)。
     *
     * @param speeds ロボット基準の速度
     */
    public void driveRobotRelative(ChassisSpeeds speeds) {
        swerveDrive.drive(speeds);
    }

    /**
     * 各モジュールを X フォーメーション (ブレーキ) にする。
     */
    public void lockWheels() {
        swerveDrive.lockPose();
    }

    // ── 状態取得 API ──────────────────────────────────────────

    /** 現在の推定 Pose を返す。 */
    public Pose2d getPose() {
        return swerveDrive.getPose();
    }

    /** IMU の Yaw を返す。 */
    public Rotation2d getYaw() {
        return swerveDrive.getYaw();
    }

    /** 現在のロボット基準 ChassisSpeeds を返す (PathPlanner 向け)。 */
    public ChassisSpeeds getRobotSpeedsMPS() {
        return swerveDrive.getRobotVelocity();
    }

    /** 全モジュールの現在位置を返す (PoseEstimator 向け)。 */
    public SwerveModulePosition[] getModulePositions() {
        return swerveDrive.getModulePositions();
    }

    /** 全モジュールの現在状態を返す。 */
    public SwerveModuleState[] getModuleStates() {
        return swerveDrive.getStates();
    }

    // ── リセット API ──────────────────────────────────────────

    /** Pose を指定値にリセットする。 */
    public void resetOdometry(Pose2d pose) {
        swerveDrive.resetOdometry(pose);
    }

    /** Gyro のヘディングをゼロリセットする。 */
    public void zeroGyro() {
        swerveDrive.zeroGyro();
    }

    /** Swerve ドライブの内部インスタンスを返す (PoseEstimator 向け)。 */
    public SwerveDrive getSwerveDrive() {
        return swerveDrive;
    }
}
