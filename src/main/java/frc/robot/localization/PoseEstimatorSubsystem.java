package frc.robot.localization;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.LocalizationConstants;
import frc.robot.subsystems.drivetrain.DrivebaseSubsystem;

/**
 * {@link SwerveDrivePoseEstimator} を用いた自己位置推定サブシステム。
 *
 * <p>毎周期 {@link #periodic()} でオドメトリを更新する。
 * Vision 観測は {@link VisionFusion} が {@link #addVisionMeasurement} を通じて注入する。
 *
 * <p>カルマンフィルタの状態量: [x, y, θ]
 * <ul>
 *   <li>オドメトリ stdDev は比較的小さく設定 (センサの短期精度が高い)</li>
 *   <li>Vision stdDev は距離依存で動的に変化 (VisionFusion が計算)</li>
 * </ul>
 */
public class PoseEstimatorSubsystem extends SubsystemBase {

    private final DrivebaseSubsystem drivetrain;
    private final SwerveDrivePoseEstimator estimator;
    private final Field2d field2d = new Field2d();

    public PoseEstimatorSubsystem(DrivebaseSubsystem drivetrain) {
        this.drivetrain = drivetrain;

        double[] oStd = LocalizationConstants.ODOMETRY_STD_DEVS;
        double[] vStd = LocalizationConstants.VISION_BASE_STD_DEVS;

        estimator = new SwerveDrivePoseEstimator(
            drivetrain.getSwerveDrive().kinematics,
            drivetrain.getYaw(),
            drivetrain.getModulePositions(),
            new Pose2d(),
            VecBuilder.fill(oStd[0], oStd[1], oStd[2]),
            VecBuilder.fill(vStd[0], vStd[1], vStd[2])
        );

        SmartDashboard.putData("PoseEstimator/Field", field2d);
    }

    @Override
    public void periodic() {
        // オドメトリ更新
        estimator.update(drivetrain.getYaw(), drivetrain.getModulePositions());

        Pose2d pose = estimator.getEstimatedPosition();
        field2d.setRobotPose(pose);

        SmartDashboard.putNumber("PoseEstimator/X",   pose.getX());
        SmartDashboard.putNumber("PoseEstimator/Y",   pose.getY());
        SmartDashboard.putNumber("PoseEstimator/Deg", pose.getRotation().getDegrees());
    }

    // ── 公開 API ──────────────────────────────────────────────

    /** 現在の推定 Pose を返す。 */
    public Pose2d getPose() {
        return estimator.getEstimatedPosition();
    }

    /**
     * Vision 測定値をカルマンフィルタに注入する。
     * {@link frc.robot.vision.VisionFusion} が適切な stdDevs を計算して呼び出す。
     *
     * @param visionPose  Vision 推定 Pose
     * @param timestamp   観測タイムスタンプ (遅延補正済み, s)
     * @param stdDevs     測定の標準偏差ベクトル [x, y, θ]
     */
    public void addVisionMeasurement(Pose2d visionPose, double timestamp,
                                     Matrix<N3, N1> stdDevs) {
        estimator.addVisionMeasurement(visionPose, timestamp, stdDevs);
    }

    /**
     * 推定 Pose を指定値にリセットする (Auto 開始時・手動ゼロ合わせ)。
     *
     * @param pose 設定する Pose
     */
    public void resetPose(Pose2d pose) {
        estimator.resetPosition(drivetrain.getYaw(), drivetrain.getModulePositions(), pose);
    }

    /** ジャイロをゼロリセットし、現在向きを 0 rad とする (POV 下ボタン用)。 */
    public void resetGyro() {
        drivetrain.zeroGyro();
    }
}
