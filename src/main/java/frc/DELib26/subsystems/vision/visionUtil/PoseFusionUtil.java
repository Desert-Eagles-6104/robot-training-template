package frc.DELib26.subsystems.vision.visionUtil;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import frc.DELib26.subsystems.vision.visionUtil.VisionFieldPoseEstimate.EstimationMethod;
import frc.DELib26.util.ProjectConstants;
import frc.DELib26.util.RobotState;

import java.util.ArrayList;
import java.util.List;
import java.util.Objects;

public class PoseFusionUtil {

	private static final record Aligned(Pose2d pose, double vx, double vy, double vyaw, int tags) {}
	/**
	 * Fuse N vision poses to the newest timestamp using inverse-variance weights.
	 * Assumes diagonal stddevs [σx, σy, σyaw]. Time-aligns each to tRef.
	 */
	public static VisionFieldPoseEstimate fuseEstimatesN(List<VisionFieldPoseEstimate> visionPoseEstimates, RobotState robotState) {
		if (visionPoseEstimates == null || visionPoseEstimates.isEmpty()) throw new IllegalArgumentException("no vision inputs");

		// Newest first
		var sorted = visionPoseEstimates.stream()
				.filter(Objects::nonNull)
				.sorted((a, b) -> Double.compare(b.getTimestampSeconds(), a.getTimestampSeconds()))
				.toList();

		double tRef = sorted.get(0).getTimestampSeconds();
		var f2rRef = robotState.getRobotPoseAt(tRef).orElse(null);
		if (f2rRef == null) return sorted.get(0);

		final double LARGE_VAR = ProjectConstants.LARGE_VARIANCE; // treat >= as ignored

		List<Aligned> aligned = alignEstimates(sorted, robotState);
		if (aligned.isEmpty()) return sorted.get(0);

		// Accumulate weights and weighted sums
		double sumWx=0, sumWy=0, sumWz=0, sumX=0, sumY=0, sumCos=0, sumSin=0;
		for (var a : aligned) {
			double wx = 1.0 / a.vx;
			double wy = 1.0 / a.vy;
			double wz = 1.0 / a.vyaw;

			sumWx += wx; sumWy += wy; sumWz += wz;
			sumX += a.pose.getX() * wx;
			sumY += a.pose.getY() * wy;
			if (wz > 0) {
				sumCos += a.pose.getRotation().getCos() * wz;
				sumSin += a.pose.getRotation().getSin() * wz;
			}
		}

		// Fallback if all weights zero
		Pose2d last = aligned.get(0).pose;
		if (sumWx==0 && sumWy==0 && sumWz==0) {
			return new VisionFieldPoseEstimate(
				last,
				tRef,
				VecBuilder.fill(Math.sqrt(LARGE_VAR), Math.sqrt(LARGE_VAR), Math.sqrt(LARGE_VAR)),
				aligned.stream().mapToInt(a -> a.tags).sum(),
				EstimationMethod.MULTI_ESTIMATE_FUSE
			);
		}
		// Fused pose
		double fx = (sumWx>0) ? sumX/sumWx : last.getX();
		double fy = (sumWy>0) ? sumY/sumWy : last.getY();
		Rotation2d fYaw = last.getRotation();

		if (!sorted.stream().anyMatch(a -> a.getVisionMeasurementStdDevs().get(2, 0)>=LARGE_VAR)) {
			fYaw = new Rotation2d(sumCos, sumSin);
		}

		Pose2d fused = new Pose2d(new Translation2d(fx, fy), fYaw);

		// Fused stddevs (per-axis)
		double sx = Math.sqrt(sumWx>0?1.0/sumWx:LARGE_VAR);
		double sy = Math.sqrt(sumWy>0?1.0/sumWy:LARGE_VAR);
		double sz = Math.sqrt(sumWz>0?1.0/sumWz:LARGE_VAR);

		Matrix<N3,N1> fusedStd = VecBuilder.fill(sx, sy, sz);
		int tags = aligned.stream().mapToInt(a -> a.tags).sum();

		return new VisionFieldPoseEstimate(fused, tRef, fusedStd, tags, EstimationMethod.MULTI_ESTIMATE_FUSE);
	}


	private static List<Aligned> alignEstimates(List<VisionFieldPoseEstimate> sortedPoseEstimates, RobotState robotState) {
		double referenceTimestamp = sortedPoseEstimates.get(0).getTimestampSeconds();
		var poseAtReferenceTimestamp = robotState.getRobotPoseAt(referenceTimestamp).orElse(null);
		if (poseAtReferenceTimestamp == null) return new ArrayList<>();

		List<Aligned> aligned = new ArrayList<>(sortedPoseEstimates.size());

		// Align and convert to variances
		for (var e : sortedPoseEstimates) {
			var robotPoseAtEstimateTimestamp = robotState.getRobotPoseAt(e.getTimestampSeconds()).orElse(null);
			if (robotPoseAtEstimateTimestamp == null) continue;

			Transform2d timeDifferenceAtReference = poseAtReferenceTimestamp.minus(robotPoseAtEstimateTimestamp);
			Pose2d pAtRef = e.getVisionRobotPoseMeters().transformBy(timeDifferenceAtReference);

			var s = e.getVisionMeasurementStdDevs();
			double vx = Math.max(Math.pow(s.get(0,0), 2), ProjectConstants.MIN_VARIANCE);
			double vy = Math.max(Math.pow(s.get(1,0), 2), ProjectConstants.MIN_VARIANCE);
			double vz = Math.pow(s.get(2,0), 2);

			aligned.add(new Aligned(pAtRef, vx, vy, vz, e.getNumTags()));
		}
		return aligned;
	}
}