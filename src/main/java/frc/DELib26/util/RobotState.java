package frc.DELib26.util;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.DELib26.interpolation.ConcurrentTimeInterpolatableBuffer;
import frc.DELib26.subsystems.swerve.SwerveIO.Pigeon2IOInputs;
import frc.DELib26.subsystems.swerve.SwerveIO.SwerveIOInputs;
import frc.DELib26.subsystems.vision.visionUtil.VisionFieldPoseEstimate;
import frc.DELib26.subsystems.vision.visionUtil.VisionSubsystemConfiguration;

import java.util.ArrayDeque;
import java.util.Deque;
import java.util.Optional;
import java.util.function.Consumer;

//A class to merge various robot state information over time for easy querying
public class RobotState {
	protected final VisionSubsystemConfiguration visionConfig;

	public static final double LOOKBACK_TIME = 1.0;

	protected final ConcurrentTimeInterpolatableBuffer<Pose2d> fieldToRobot =
			ConcurrentTimeInterpolatableBuffer.createBuffer(LOOKBACK_TIME);
	protected final ConcurrentTimeInterpolatableBuffer<Double> driveYawAngularVelocity =
			ConcurrentTimeInterpolatableBuffer.createDoubleBuffer(LOOKBACK_TIME);
	protected final ConcurrentTimeInterpolatableBuffer<Double> driveRollAngularVelocity =
			ConcurrentTimeInterpolatableBuffer.createDoubleBuffer(LOOKBACK_TIME);
	protected final ConcurrentTimeInterpolatableBuffer<Double> drivePitchAngularVelocity =
			ConcurrentTimeInterpolatableBuffer.createDoubleBuffer(LOOKBACK_TIME);

	protected final ConcurrentTimeInterpolatableBuffer<Double> robotSpeedMagnitude =
			ConcurrentTimeInterpolatableBuffer.createDoubleBuffer(LOOKBACK_TIME);


	private final Deque<VisionFieldPoseEstimate> pendingVision = new ArrayDeque<>();
	protected Optional<VisionFieldPoseEstimate> lastVisionMeasurement = Optional.empty();
	private boolean useVision = true;
	// For MG2 yaw anchoring
	protected double lastYawAnchorTs = Double.NaN;
	protected Optional<Rotation2d> lastYawAnchorYaw = Optional.empty();

	public RobotState(VisionSubsystemConfiguration visionConfig) {
		this.visionConfig = visionConfig;
		this.fieldToRobot.addSample(0.0, new Pose2d());
	}

	public void updateFromSwerveInputs(SwerveIOInputs swerveinputs, Pigeon2IOInputs pigeon2Inputs) {

		this.fieldToRobot.addSample(swerveinputs.Timestamp, swerveinputs.Pose);
		this.robotSpeedMagnitude.addSample(
			swerveinputs.Timestamp,
			Math.hypot(swerveinputs.RobotSpeeds.vxMetersPerSecond, swerveinputs.RobotSpeeds.vyMetersPerSecond)
		);

		this.driveYawAngularVelocity.addSample(
			pigeon2Inputs.timestamp,
			Math.toRadians(pigeon2Inputs.angularYawVelocityDPS)
		);
		this.driveRollAngularVelocity.addSample(
			pigeon2Inputs.timestamp,
			Math.toRadians(pigeon2Inputs.angularRollVelocityDPS)
		);
		this.drivePitchAngularVelocity.addSample(
			pigeon2Inputs.timestamp,
			Math.toRadians(pigeon2Inputs.angularPitchVelocityDPS)
		);
	}

	public Pose2d getLatestRobotPose() {
		return this.fieldToRobot.getLatest().getValue();
	}

	public double getLatestRobotPoseTimestamp() {
		return this.fieldToRobot.getLatest().getKey();
	}

	public Optional<Pose2d> getRobotPoseAt(double timestampSeconds) {
		return fieldToRobot.getSample(timestampSeconds);
	}

	public Optional<Double> getRobotSpeedMagnitudeAt(double timestampSeconds) {
		return robotSpeedMagnitude.getSample(timestampSeconds);
	}

	public Optional<Double> getMaxAbsDriveYawAngularVelocityInRange(double startTime, double endTime) {
		return getMaxAbsValueInRange(this.driveYawAngularVelocity, startTime, endTime);
	}

	public void setUseVision(boolean useVision) {
		if (this.useVision && !useVision) {
			this.lastVisionMeasurement = Optional.empty();
			this.pendingVision.clear();
		}
		this.useVision = useVision;
	}

	public double getLastYawAnchorTs() {
		return this.lastYawAnchorTs;
	}

	public Optional<Rotation2d> getLastYawAnchorYaw() {
		return this.lastYawAnchorYaw;
	}

	public void setLastYawAnchor(Rotation2d yaw, double currentTimestampSeconds) {
		if (!this.useVision) { return; }
		this.lastYawAnchorYaw = Optional.of(yaw);
		this.lastYawAnchorTs = currentTimestampSeconds;
	}

	public void invalidateYawAnchor() {
		this.lastYawAnchorYaw = Optional.empty();
		this.lastYawAnchorTs = Double.NaN;
	}

	public Optional<VisionFieldPoseEstimate> getLatestMegatagEstimate() {
		return this.lastVisionMeasurement;
	}

	/** Drain all new estimates since last periodic. */
	public void forEachPendingMegatagEstimate(Consumer<VisionFieldPoseEstimate> consumer) {
		while (!pendingVision.isEmpty()) {
			consumer.accept(pendingVision.removeFirst());
		}
	}

	public void updateMegatagEstimate(VisionFieldPoseEstimate megatagEstimate) {
		if (!this.useVision) { return; }
		this.evaluatePoseEstimate(megatagEstimate);
		this.lastVisionMeasurement = Optional.of(megatagEstimate);
		this.pendingVision.addLast(megatagEstimate);
	}

	private boolean evaluatePoseEstimate(VisionFieldPoseEstimate estimate) {
		Pose2d visionPose = estimate.getVisionRobotPoseMeters();
		double ts = estimate.getTimestampSeconds();

		Optional<Pose2d> odomOpt = this.getRobotPoseAt(ts);
		if (odomOpt.isEmpty()) {
			// nothing to compare, just accept
			return true;
		}

		Pose2d odomPose = odomOpt.get();
		double dist = odomPose.getTranslation().getDistance(visionPose.getTranslation());
		double yawDiff = Math.abs(
			odomPose.getRotation().minus(visionPose.getRotation()).getDegrees()
		);

		boolean isMovementValid = dist <= this.visionConfig.maxTranslationJumpMeters
			&& yawDiff <= this.visionConfig.maxYawJumpDegrees;

		// If valid, no special handling needed
		if (isMovementValid) {
			return true;
		}
		// -> down-weight the measurement by inflating std devs.
		double stdX = Math.abs(odomPose.getX() - visionPose.getX());
		double stdY = Math.abs(odomPose.getY() - visionPose.getY());
		double stdTheta = Math.abs(
			odomPose.getRotation().getRadians() - visionPose.getRotation().getRadians()
		);

		estimate.setVisionMeasurementStdDevs(
			VecBuilder.fill(stdX, stdY, stdTheta)
		);

		// Measurement is allowed, just low-trust
		return true;
	}

	private static Optional<Double> getMaxAbsValueInRange(ConcurrentTimeInterpolatableBuffer<Double> buffer, double minTime, double maxTime) {
		if (maxTime < minTime) return Optional.empty();
		var submap = buffer.getInternalBuffer().subMap(minTime, maxTime).values();
		var max = submap.stream().max(Double::compare);
		var min = submap.stream().min(Double::compare);
		if (max.isEmpty() || min.isEmpty()) return Optional.empty();
		if (Math.abs(max.get()) >= Math.abs(min.get())) return max;
		else return min;
	}

}