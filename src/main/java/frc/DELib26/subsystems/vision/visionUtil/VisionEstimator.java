package frc.DELib26.subsystems.vision.visionUtil;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.DELib26.booleanUtil.LatchedBoolean;
import frc.DELib26.subsystems.vision.IOs.CameraIO.CameraIOInputs;
import frc.DELib26.util.FieldUtil;
import frc.DELib26.util.RobotState;

import java.util.ArrayList;
import java.util.List;
import java.util.Optional;

public class VisionEstimator {
	private final RobotState robotState;
	private final VisionSubsystemConfiguration config;
	private final EstimateProcessor estimateProcessor;

	private VisionFieldPoseEstimate lastAcceptedEstimate = null;
	private final List<Optional<VisionFieldPoseEstimate>> latestProcessedCameraEstimates;

	// Latched gate that enables MG2 once we have a valid yaw anchor from MT1
	// and the robot heading has remained stable/converged since that anchor.
	private final LatchedBoolean mg2EnableLatch = new LatchedBoolean();

	public VisionEstimator(VisionSubsystemConfiguration config, RobotState robotState) {
		this.config = config;
		this.robotState = robotState;
		this.estimateProcessor = new EstimateProcessor(config, robotState);

		int numCameras = config.cameraConfigurations.length;
		this.latestProcessedCameraEstimates = new ArrayList<>(numCameras);
		for (int i = 0; i < numCameras; i++) {
			this.latestProcessedCameraEstimates.add(Optional.empty());
		}
	}

	public void processCameraEstimates(CameraIOInputs[] camsSnapshot) {
		List<VisionFieldPoseEstimate> estimates = new ArrayList<>();
		for (int i = 0; i < camsSnapshot.length; i++) {
			Optional<VisionFieldPoseEstimate> estimate = this.processCamera(camsSnapshot[i]);

			this.latestProcessedCameraEstimates.set(i, estimate);
			estimate.ifPresent(estimates::add);
		}

		// if no estimates, do nothing
		// if one estimate, accept it
		// if multiple estimates, fuse them
		Optional<VisionFieldPoseEstimate> accepted = Optional.empty();
		if (estimates.size() == 1) {
			accepted = Optional.of(estimates.get(0));
		} else if (estimates.size() > 1) {
			accepted = Optional.of(PoseFusionUtil.fuseEstimatesN(estimates, this.robotState));
		}

		accepted.ifPresent(est -> {
			this.robotState.updateMegatagEstimate(est);
			this.lastAcceptedEstimate = est;
		});
	}

	/**
	 * Process a single camera's inputs into a vision pose estimate.
	 *
	 * @param cam The camera inputs.
	 * @return The processed vision pose estimate, if any.
	 */
	public Optional<VisionFieldPoseEstimate> processCamera(CameraIOInputs cam) {
		if (!cam.seesTarget) {
			return Optional.empty();
		}

		MegatagPoseEstimate mt1 = cam.megatag1PoseEstimate;
		MegatagPoseEstimate mt2 = cam.megatag2PoseEstimate;

		Optional<VisionFieldPoseEstimate> primaryMt1 = tryPrimaryMt1(mt1);
		if (primaryMt1.isPresent()) {
			updateYawAnchorFromMt1(primaryMt1.get());
			return primaryMt1;
		}

		refreshMg2State();

		return tryMt2(mt2);
	}

	private Optional<VisionFieldPoseEstimate> tryPrimaryMt1(MegatagPoseEstimate mt1) {
		if (!isFresh(mt1)) {
			return Optional.empty();
		}

		return estimateProcessor.processMegatag1PoseEstimate(mt1);
	}

	private Optional<VisionFieldPoseEstimate> tryMt2(MegatagPoseEstimate mt2) {
		if (!canUseMt2(mt2)) {
			return Optional.empty();
		}

		return estimateProcessor.processMegatag2PoseEstimate(mt2);
	}

	private void updateYawAnchorFromMt1(VisionFieldPoseEstimate estimate) {
		if (mg2EnableLatch.getAsBoolean()) {
			return;
		}
		robotState.setLastYawAnchor(estimate.getVisionRobotPoseMeters().getRotation(), estimate.getTimestampSeconds());
	}

	private void refreshMg2State() {
		if (!hasYawAnchor()) {
			mg2EnableLatch.reset();
			return;
		}

		if (hasExceededYawLimitSinceAnchor()) {
			this.invalidateYawAnchor();
			return;
		}

		mg2EnableLatch.update(isYawConvergedToAnchor());
	}

	private boolean canUseMt2(MegatagPoseEstimate mt2) {
		return mt2 != null
				&& isFresh(mt2)
				&& mg2EnableLatch.getAsBoolean()
				&& hasYawAnchor()
				&& mt2.timestampSeconds > robotState.getLastYawAnchorTs();
	}

	private boolean hasYawAnchor() {
		return Double.isFinite(robotState.getLastYawAnchorTs());
	}

	private boolean hasExceededYawLimitSinceAnchor() {
		double maxOmega = robotState.getMaxAbsDriveYawAngularVelocityInRange(
				robotState.getLastYawAnchorTs(),
				robotState.getLatestRobotPoseTimestamp()).orElse(Double.POSITIVE_INFINITY);

		return maxOmega > config.maxAllowedYawAngularVelocityMegatag2;
	}

	private void invalidateYawAnchor() {
		robotState.invalidateYawAnchor();
		mg2EnableLatch.reset();
	}

	private boolean isYawConvergedToAnchor() {
		Pose2d nowPose = robotState.getLatestRobotPose();
		Optional<Rotation2d> optionalAnchorYaw = robotState.getLastYawAnchorYaw();
		if (optionalAnchorYaw.isEmpty()) {
			return false; // cant be converged if we dont have an anchor
		}
		double diffDeg = FieldUtil.getAngleDifferenceDegrees(nowPose.getRotation(), optionalAnchorYaw.get());
		return diffDeg <= config.mg2ConvergenceDegs;
	}

	private boolean isFresh(MegatagPoseEstimate estimate) {
		return estimate != null
				&& (lastAcceptedEstimate == null
						|| estimate.timestampSeconds > lastAcceptedEstimate.getTimestampSeconds());
	}

	public Optional<VisionFieldPoseEstimate> getLastAcceptedEstimate() {
		return Optional.ofNullable(this.lastAcceptedEstimate);
	}

	public List<Optional<VisionFieldPoseEstimate>> getLatestProcessedCameraEstimates() {
		return this.latestProcessedCameraEstimates;
	}

	public boolean isMg2Enabled() {
		return this.mg2EnableLatch.getAsBoolean();
	}

}
