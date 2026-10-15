package frc.DELib26.subsystems.vision.visionUtil;

import com.ctre.phoenix6.Utils;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import frc.DELib26.util.LoggerUtil;
import frc.DELib26.util.vision.LimelightHelpers;
import frc.DELib26.util.vision.LimelightHelpers.PoseEstimate;
import frc.DELib26.util.vision.LimelightHelpers.RawFiducial;
import org.littletonrobotics.junction.Logger;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;
import java.util.Objects;
//TODO: Unfinished work. Needs to be tuned on robot.
//Immutable by design. That's why we copy everything in the constructor
public class MegatagPoseEstimate {
	public final double timestampSeconds;
	public final Pose2d bluePose;
	public final Pose3d bluePose3d;
	public final double latency;
	public final double avgTagArea;
	public final double quality;
	public final int[] fiducialIds;
	public final LimelightHelpers.RawFiducial[] fiducialObservations;
	// [MT1x, MT1y, MT1z, MT1roll, MT1pitch, MT1Yaw, MT2x, MT2y, MT2z, MT2roll, MT2pitch, MT2yaw]
	public final double[] standardDeviations;

	public MegatagPoseEstimate(Boolean seesTarget, PoseEstimate poseEstimate, double[] standardDeviations, Pose3d robotPose3d) {
		int[] ids = Arrays.stream(poseEstimate.rawFiducials)
			.filter(Objects::nonNull)
			.mapToInt(f -> f.id())
			.toArray();

		// Build obs list AFTER basic timestamp filter
		List<RawFiducial> obs = Arrays.stream(poseEstimate.rawFiducials)
			.filter(Objects::nonNull)
			.toList();

		// If Limelight gave us a pose but no usable fiducials, treat as invalid
		if (obs.isEmpty()) {
			throw new IllegalArgumentException("No valid fiducials in pose estimate");
		}

		this.timestampSeconds = Utils.fpgaToCurrentTime(poseEstimate.timestampSeconds);

		this.fiducialIds = Arrays.copyOf(ids, ids.length);;
		this.latency = poseEstimate.latency;
		this.avgTagArea = poseEstimate.avgTagArea;
		this.quality = computeQuality(obs);
		this.fiducialObservations = Arrays.stream(poseEstimate.rawFiducials)
			.filter(Objects::nonNull)
			.map(LimelightHelpers::copyRawFiducial)
			.toArray(RawFiducial[]::new);

		// 2D + 3D poses from this frame
		this.bluePose = poseEstimate.pose;
		this.bluePose3d = robotPose3d;

		// Stddevs from readCameraData()
		this.standardDeviations = Arrays.copyOf(
			standardDeviations,
			standardDeviations.length
		);;
	}

	private static double computeQuality(List<RawFiducial> obs) {
		if (obs.isEmpty()) return 0.0;

		List<RawFiducial> tags = new ArrayList<>(obs);
		tags.sort((a, b) -> Double.compare(singleTagQuality(b), singleTagQuality(a)));

		List<RawFiducial> used = new ArrayList<>();

		RawFiducial best = tags.get(0);
		used.add(best);

		double quality = singleTagQuality(best);

		for (int i = 1; i < tags.size(); i++) {
			RawFiducial tag = tags.get(i);

			double tagScore = singleTagQuality(tag);
			double geometryScore = geometryScore(tag, used);

			double diminishingWeight = 1.0 / Math.sqrt(i + 1.0);

			double contribution = tagScore * geometryScore * diminishingWeight;
			quality += contribution;

			used.add(tag);
		}

		return clamp01(quality);
	}

	private static double singleTagQuality(RawFiducial tag) {
		return 1.0 - clamp01(tag.ambiguity());
	}

	private static double geometryScore(RawFiducial candidate, List<RawFiducial> usedTags) {
		if (usedTags == null || usedTags.isEmpty()) {
			return 0.0;
		}

		final double X_WEIGHT = 1.0;
		final double Y_WEIGHT = 0.35;
		final double GOOD_TAG_SEPARATION = 15.0;

		double bestSeparation = 0.0;

		for (RawFiducial used : usedTags) {
			double dx = candidate.txnc() - used.txnc();
			double dy = candidate.tync() - used.tync();

			double separation = Math.hypot(dx * X_WEIGHT, dy * Y_WEIGHT);
			bestSeparation = Math.max(bestSeparation, separation);
		}

		return clamp01(bestSeparation / GOOD_TAG_SEPARATION);
	}

	private static double clamp01(double x) {
		return MathUtil.clamp(x, 0.0, 1.0);
	}

	public static void log(MegatagPoseEstimate estimate, String prefix) {
		if (estimate == null) {
			Logger.recordOutput(prefix + "/bluePose", LoggerUtil.NO_POSE_2D);
			Logger.recordOutput(prefix + "/bluePose3d", LoggerUtil.NO_POSE_3D);
			Logger.recordOutput(prefix + "/timestampSeconds", LoggerUtil.NO_DOUBLE);
			Logger.recordOutput(prefix + "/latency", LoggerUtil.NO_DOUBLE);
			Logger.recordOutput(prefix + "/avgTagArea", LoggerUtil.NO_DOUBLE);
			Logger.recordOutput(prefix + "/quality", LoggerUtil.NO_DOUBLE);
			Logger.recordOutput(prefix + "/fiducialIds", new int[] {});
			Logger.recordOutput(prefix + "/standardDeviations", new double[] {});

			Logger.recordOutput(prefix + "/fiducialObservations", new RawFiducial[] {});
		} else {
			Logger.recordOutput(prefix + "/bluePose", estimate.bluePose);
			Logger.recordOutput(prefix + "/bluePose3d", estimate.bluePose3d);
			Logger.recordOutput(prefix + "/timestampSeconds", estimate.timestampSeconds);
			Logger.recordOutput(prefix + "/latency", estimate.latency);
			Logger.recordOutput(prefix + "/avgTagArea", estimate.avgTagArea);
			Logger.recordOutput(prefix + "/quality", estimate.quality);
			Logger.recordOutput(prefix + "/fiducialIds", estimate.fiducialIds);
			Logger.recordOutput(prefix + "/standardDeviations", estimate.standardDeviations);

			Logger.recordOutput(prefix + "/fiducialObservations", estimate.fiducialObservations);
		}
	}
}
