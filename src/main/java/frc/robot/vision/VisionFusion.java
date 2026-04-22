package frc.robot.vision;

import edu.wpi.first.math.MatBuilder;
import edu.wpi.first.math.Nat;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.LocalizationConstants;
import frc.robot.localization.PoseEstimatorSubsystem;

/**
 * Limelight の Vision 測定値をゲーティングして {@link PoseEstimatorSubsystem} へ
 * 注入する融合レイヤー。
 *
 * <p>測定ゲーティング条件:
 * <ul>
 *   <li>タグが 1 枚以上検出されていること</li>
 *   <li>推定 Pose と Vision Pose のジャンプが {@code MAX_POSE_JUMP} 未満であること</li>
 * </ul>
 *
 * <p>標準偏差 (stdDevs) を距離に応じて動的に増大させることで、
 * 遠距離での低信頼度観測が Kalman ゲインに影響を与えすぎないようにする:
 * <pre>
 *   σ = σ_base × (1 + avgDist / DIST_SCALE)
 * </pre>
 */
public class VisionFusion extends SubsystemBase {

    private final PoseEstimatorSubsystem poseEstimator;
    private final LimelightSubsystem     limelight;

    public VisionFusion(PoseEstimatorSubsystem poseEstimator, LimelightSubsystem limelight) {
        this.poseEstimator = poseEstimator;
        this.limelight     = limelight;
    }

    @Override
    public void periodic() {
        limelight.getLatestMeasurement().ifPresent(this::tryInject);
    }

    // ── 内部 ──────────────────────────────────────────────────

    private void tryInject(VisionMeasurement m) {
        // ゲーティング 1: タグ未検出
        if (m.tagCount() < 1) return;

        // ゲーティング 2: 急激な位置ジャンプ
        double jump = poseEstimator.getPose()
            .getTranslation()
            .getDistance(m.pose().getTranslation());
        if (jump > LocalizationConstants.MAX_POSE_JUMP) return;

        // stdDevs の距離依存スケーリング
        double scale = 1.0 + m.avgDist() / LocalizationConstants.VISION_DIST_SCALE;
        double[] base = LocalizationConstants.VISION_BASE_STD_DEVS;
        var stdDevs = MatBuilder.fill(Nat.N3(), Nat.N1(),
            base[0] * scale,
            base[1] * scale,
            base[2] * scale
        );

        poseEstimator.addVisionMeasurement(m.pose(), m.timestamp(), stdDevs);
    }
}
