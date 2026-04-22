package frc.robot.vision;

import edu.wpi.first.math.geometry.Pose2d;

/**
 * Limelight 4 から取得した単一フレームの Vision 測定値を格納する値クラス。
 *
 * @param pose      AprilTag 観測から算出したフィールド基準の Pose 推定値
 * @param timestamp 観測時のロボット側タイムスタンプ (s) — 遅延補正済み
 * @param tagCount  観測に使用した AprilTag の数
 * @param avgDist   タグまでの平均距離 (m)
 */
public record VisionMeasurement(
    Pose2d pose,
    double timestamp,
    int    tagCount,
    double avgDist
) {}
