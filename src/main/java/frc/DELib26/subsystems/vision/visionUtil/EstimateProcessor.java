package frc.DELib26.subsystems.vision.visionUtil;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.DriverStation;
import frc.DELib26.subsystems.vision.visionUtil.VisionFieldPoseEstimate.EstimationMethod;
import frc.DELib26.util.RobotState;

import java.util.Optional;

public class EstimateProcessor {
	private final VisionSubsystemConfiguration config;
	private final RobotState robotState;

	public EstimateProcessor(VisionSubsystemConfiguration config, RobotState robotState) {
		this.config = config;
		this.robotState = robotState;
	}

	public Optional<VisionFieldPoseEstimate> processMegatag1PoseEstimate(MegatagPoseEstimate estimate) {
		if (!this.passesBaseFilters(estimate)) {
			return Optional.empty();
		}

		if (estimate.quality < this.config.minMegatag1Quality) {
			return Optional.empty();
		}

		double scaleFactor = 1.0 / estimate.quality;
		Matrix<N3, N1> visionStdDevs = this.getPose2dStd(estimate, scaleFactor);
		Optional<Matrix<N3, N1>> maybeAdjustedStdDevs = this.calculatePoseDiscrepancy(estimate, visionStdDevs);

		return maybeAdjustedStdDevs.map(adjustedStdDevs ->
			new VisionFieldPoseEstimate(
					estimate.bluePose,
					estimate.timestampSeconds,
					adjustedStdDevs,
					estimate.fiducialIds.length,
					EstimationMethod.MEGA_TAG1
				)
			);
	}

	/**
	 * Build a VisionFieldPoseEstimate from a MegaTag2 solve.
	 *
	 * Assumptions:
	 * - The heading in mg2Pose already uses our drive heading as a prior,
	 *   so we normally TRUST the rotation and do not compare it to MT1/gyro.
	 * - We still run basic geometric sanity checks and exclusive-tag filtering.
	 * - The 12 standard deviations are split into six for MegaTag1 and six for MegaTag2.
	 *   Here, we are accessing the second block, which corresponds to MegaTag2.
	 */
	public Optional<VisionFieldPoseEstimate> processMegatag2PoseEstimate(MegatagPoseEstimate estimate) {
		if (!this.passesBaseFilters(estimate)) {
			return Optional.empty();
		}

		if (estimate.quality < this.config.minMegatag2Quality) {
			return Optional.empty();
		}

		// MegaTag2 uses gyro heading as prior, so rotation is already fused
		double xyStd = this.getXYStd(estimate);
		Optional<Matrix<N3, N1>> maybeAdjustedStdDevs = this.calculatePoseDiscrepancy(
			estimate,
			VecBuilder.fill(xyStd, xyStd, this.config.largeVariance)
		);

		return maybeAdjustedStdDevs.map(adjustedStdDevs ->
			new VisionFieldPoseEstimate(
				estimate.bluePose,
				estimate.timestampSeconds,
				adjustedStdDevs,
				estimate.fiducialIds.length,
				EstimationMethod.MEGA_TAG2
			)
		);
	}

	private boolean passesBaseFilters(MegatagPoseEstimate estimate) {
		boolean isExcessiveYaw = this.isYawVelocityExceedingLimit(estimate.timestampSeconds);
		boolean isAboveHighSpeedThreshold = robotState.getRobotSpeedMagnitudeAt(estimate.timestampSeconds)
			.orElse(Double.POSITIVE_INFINITY) > this.config.highSpeedThresholdMetersPerSecond;
		boolean isZThresholdExceeded = Math.abs(estimate.bluePose3d.getZ()) > this.config.defaultZThreshold;
		return !isExcessiveYaw && !isZThresholdExceeded && !isAboveHighSpeedThreshold;
	}

	private boolean isYawVelocityExceedingLimit(double timestampSeconds){
		return this.robotState.getMaxAbsDriveYawAngularVelocityInRange(
			timestampSeconds - this.config.highYawLookbackS,
			timestampSeconds
		).orElse(Double.POSITIVE_INFINITY) > this.config.highYawVelocityRadS;
	}

	private Optional<Matrix<N3, N1>> calculatePoseDiscrepancy(MegatagPoseEstimate estimate, Matrix<N3, N1> stdDevs) {
		Pose2d visionPose = estimate.bluePose;
		double ts = estimate.timestampSeconds;

		Optional<Pose2d> odomOpt = this.robotState.getRobotPoseAt(ts);
		if (odomOpt.isEmpty()) {
			// nothing to compare, just accept
			return Optional.of(stdDevs);
		}

		Pose2d odomPose = odomOpt.get();
		double dist = odomPose.getTranslation().getDistance(visionPose.getTranslation());
		double yawDiff = Math.abs(
			odomPose.getRotation().minus(visionPose.getRotation()).getDegrees()
		);

		boolean isMovementValid = dist <= this.config.maxTranslationJumpMeters
			&& yawDiff <= this.config.maxYawJumpDegrees;

		if (isMovementValid) {
			return Optional.of(stdDevs);
		}

		// If movement is invalid but we're disabled, still accept (allows resetting odometry with vision while disabled)
		if (DriverStation.isEnabled()) {
			return Optional.empty();
		}

		double stdX = Math.abs(odomPose.getX() - visionPose.getX());
		double stdY = Math.abs(odomPose.getY() - visionPose.getY());
		double stdTheta = Math.abs(
			odomPose.getRotation().getRadians() - visionPose.getRotation().getRadians()
		);

		return Optional.of(VecBuilder.fill(stdX, stdY, stdTheta));
	}


	public double getXYStd(MegatagPoseEstimate estimate) {
		return this.getXYStd(estimate, 1.0);
	}

	public double getXYStd(MegatagPoseEstimate estimate, double scaleFactor) {
		double xStd = estimate.standardDeviations[VisionSubsystemConfiguration.MEGATAG_X_STD_DEV_INDEX] * scaleFactor;
		double yStd = estimate.standardDeviations[VisionSubsystemConfiguration.MEGATAG_Y_STD_DEV_INDEX] * scaleFactor;
		return Math.max(xStd, yStd);
	}

	public Matrix<N3, N1> getPose2dStd(MegatagPoseEstimate estimate, double scaleFactor) {
		double xyStd = this.getXYStd(estimate, scaleFactor);
		double rotStd = estimate.standardDeviations[VisionSubsystemConfiguration.MEGATAG_YAW_STD_DEV_INDEX];
		return VecBuilder.fill(xyStd, xyStd, rotStd);
	}

}
