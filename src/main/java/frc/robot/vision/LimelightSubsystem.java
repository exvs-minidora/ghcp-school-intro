package frc.robot.vision;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.VisionConstants;

import java.util.Optional;

/**
 * Limelight 4 から AprilTag 観測データを取得し {@link VisionMeasurement} に
 * 変換するサブシステム。
 *
 * <p>botpose_wpiblue (Megatag2) を主として使用する。
 * タイムスタンプ補正を適用し、取得失敗 / 無効フレーム時は空 Optional を返す。
 */
public class LimelightSubsystem extends SubsystemBase {

    private final NetworkTable table;

    // NT キー
    private static final String KEY_BOTPOSE  = "botpose_wpiblue";
    private static final String KEY_TV        = "tv";
    private static final String KEY_LATENCY   = "tl";
    private static final String KEY_PIPELINE_LATENCY = "cl";
    private static final String KEY_TID       = "tid";

    /** 最後に取得した測定値 (存在しない場合は empty) */
    private Optional<VisionMeasurement> latestMeasurement = Optional.empty();

    public LimelightSubsystem() {
        table = NetworkTableInstance.getDefault()
            .getTable(VisionConstants.LIMELIGHT_NAME);
    }

    @Override
    public void periodic() {
        latestMeasurement = parseMeasurement();

        SmartDashboard.putBoolean("Vision/HasTarget",
            latestMeasurement.isPresent());
        latestMeasurement.ifPresent(m -> {
            SmartDashboard.putNumber("Vision/PoseX",   m.pose().getX());
            SmartDashboard.putNumber("Vision/PoseY",   m.pose().getY());
            SmartDashboard.putNumber("Vision/TagCount", m.tagCount());
            SmartDashboard.putNumber("Vision/AvgDist",  m.avgDist());
        });
    }

    // ── 公開 API ──────────────────────────────────────────────

    /**
     * 最新の有効な Vision 測定値を返す。
     * ターゲット非検出時または無効フレーム時は {@link Optional#empty()} を返す。
     */
    public Optional<VisionMeasurement> getLatestMeasurement() {
        return latestMeasurement;
    }

    // ── 内部 ──────────────────────────────────────────────────

    private Optional<VisionMeasurement> parseMeasurement() {
        // tv (有効ターゲット) が 1 でなければスキップ
        if (table.getEntry(KEY_TV).getDouble(0.0) < 0.5) {
            return Optional.empty();
        }

        // botpose_wpiblue: [x, y, z, rx, ry, rz(deg), latency_ms, tagCount, tagSpan, avgDist, avgArea]
        double[] botpose = table.getEntry(KEY_BOTPOSE)
            .getDoubleArray(new double[0]);
        if (botpose.length < 7) {
            return Optional.empty();
        }

        // 遅延補正 (ms → s)
        double captureLatency  = table.getEntry(KEY_LATENCY).getDouble(VisionConstants.LATENCY_FALLBACK_S * 1000.0);
        double pipelineLatency = table.getEntry(KEY_PIPELINE_LATENCY).getDouble(0.0);
        double totalLatencySec = (captureLatency + pipelineLatency) / 1000.0;
        double timestamp       = Timer.getFPGATimestamp() - totalLatencySec;

        Pose2d pose = new Pose2d(
            new Translation2d(botpose[0], botpose[1]),
            Rotation2d.fromDegrees(botpose[5])
        );

        int    tagCount = (botpose.length > 7) ? (int) botpose[7] : 1;
        double avgDist  = (botpose.length > 9) ? botpose[9]       : 0.0;

        return Optional.of(new VisionMeasurement(pose, timestamp, tagCount, avgDist));
    }
}
