package frc.DELib26.subsystems.vision.visionUtil;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;

/** Represents a robot pose estimate from vision with associated uncertainty and metadata. */
public class VisionFieldPoseEstimate {

	private final Pose2d visionRobotPoseMeters;
	private final double timestampSeconds;
	private Matrix<N3, N1> visionMeasurementStdDevs;
	private final int numTags;
	private final EstimationMethod estimationMethod;

	/**
	 * Creates a new vision field pose estimate.
	 *
	 * @param visionRobotPoseMeters The estimated robot pose on the field in meters
	 * @param timestampSeconds The timestamp when this estimate was captured
	 * @param visionMeasurementStdDevs Standard deviations representing measurement uncertainty
	 * @param numTags Number of AprilTags used in this pose estimate
	 */
	public VisionFieldPoseEstimate(
			Pose2d visionRobotPoseMeters,
			double timestampSeconds,
			Matrix<N3, N1> visionMeasurementStdDevs,
			int numTags,
			EstimationMethod estimationMethod
		) {
		this.visionRobotPoseMeters = visionRobotPoseMeters;
		this.timestampSeconds = timestampSeconds;
		this.visionMeasurementStdDevs = visionMeasurementStdDevs;
		this.numTags = numTags;
		this.estimationMethod = estimationMethod;
	}

	public void setVisionMeasurementStdDevs(Matrix<N3, N1> stdDevs) {
		this.visionMeasurementStdDevs = stdDevs;
	}

	public Pose2d getVisionRobotPoseMeters() {
		return visionRobotPoseMeters;
	}

	public double getTimestampSeconds() {
		return timestampSeconds;
	}

	public Matrix<N3, N1> getVisionMeasurementStdDevs() {
		return visionMeasurementStdDevs;
	}

	public int getNumTags() {
		return numTags;
	}

	public EstimationMethod getEstimationMethod() {
		return estimationMethod;
	}

	/** Which vision pipeline produced the estimate. */
	public static enum EstimationMethod {
		/** Limelight MegaTag1 pipeline. */
		MEGA_TAG1,
		/** Limelight MegaTag2 pipeline. */
		MEGA_TAG2,
		/** Multiple estimates/cameras fused into one. */
		MULTI_ESTIMATE_FUSE,
		/** Default/unspecified. */
		UNKNOWN
	}
}