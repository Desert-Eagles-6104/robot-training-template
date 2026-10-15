package frc.DELib26.subsystems.vision;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.wpilibj.DriverStation;
import frc.DELib26.subsystems.vision.IOs.CameraIO;
import frc.DELib26.subsystems.vision.IOs.CameraIO.CameraIOInputs;
import frc.DELib26.subsystems.vision.IOs.CameraIOHardwareLimelight;
import frc.DELib26.subsystems.vision.IOs.VisionIO;
import frc.DELib26.subsystems.vision.IOs.VisionIO.VisionIOInputs;
import frc.DELib26.subsystems.vision.IOs.VisionIOHardwareLimelight;
import frc.DELib26.subsystems.vision.IOs.VisionIOSimPhoton;
import frc.DELib26.subsystems.vision.visionUtil.MegatagPoseEstimate;
import frc.DELib26.subsystems.vision.visionUtil.VisionEstimator;
import frc.DELib26.subsystems.vision.visionUtil.VisionFieldPoseEstimate;
import frc.DELib26.subsystems.vision.visionUtil.VisionSubsystemConfiguration;
import frc.DELib26.util.LoggerUtil;
import frc.DELib26.util.RobotState;
import frc.DELib26.util.RunMode;
import frc.DELib26.util.SubsystemDataProcessor;
import frc.DELib26.util.VirtualSubsystem;
import org.littletonrobotics.junction.Logger;

import java.util.ArrayList;
import java.util.HashSet;
import java.util.List;
import java.util.Optional;
import java.util.Set;

// new subsystem for objict detection using limelights
// http://limelight-<name>.local:5801/
public abstract class VisionSubsystem extends VirtualSubsystem {

	private final VisionIO visionIO;
	private final VisionIOInputs visionInputs;
	private final VisionSubsystemConfiguration config;

	private final VisionEstimator visionEstimator;

	private final Object visionIOLock = new Object();
	private final boolean useThread;
	private boolean lastDisabled = false;

	public VisionSubsystem(VisionSubsystemConfiguration config, RobotState robotState) {
		this.config = config;

		this.visionEstimator = new VisionEstimator(config, robotState);

		int numCameras = config.cameraConfigurations.length;
		CameraIO[] cameraIOs = new CameraIO[numCameras];

		for (int i = 0; i < numCameras; i++) {
			cameraIOs[i] = new CameraIOHardwareLimelight(
					config.cameraConfigurations[i],
					() -> robotState.getLatestRobotPose().getRotation()
							.getDegrees());
		}

		if (RunMode.isSim()) {
			this.visionIO = new VisionIOSimPhoton(cameraIOs, config, () -> robotState.getLatestRobotPose());
		} else {
			this.visionIO = new VisionIOHardwareLimelight(cameraIOs, config);
		}

		this.visionInputs = new VisionIOInputs(numCameras);
		this.useThread = true;
		if (this.useThread) {
			SubsystemDataProcessor.createAndStartSubsystemDataProcessor(
					() -> {
						synchronized (visionIOLock) {
							visionIO.updateInputs(visionInputs);
						}
					},
					visionIO,
					"VisionIO");
		}
	}

	private CameraIOInputs[] getCameraInputsSnapshot() {
		if (this.useThread) {
			synchronized (visionIOLock) {
				CameraIOInputs[] camsSnapshot = new CameraIOInputs[this.visionInputs.cameraInputs.length];
				for (int i = 0; i < camsSnapshot.length; i++) {
					camsSnapshot[i] = this.visionInputs.cameraInputs[i].copy();
				}
				return camsSnapshot;
			}
		} else {
			return this.visionInputs.cameraInputs;
		}
	}

	private void setCameraThrottle(int throttle) {
		if (this.useThread) {
			synchronized (visionIOLock) {
				this.visionIO.setCameraThrottle(throttle);
			}
		} else {
			this.visionIO.setCameraThrottle(throttle);
		}
	}

	@Override
	public void periodic() {
		boolean currentlyDisabled = DriverStation.isDisabled();
		if (lastDisabled && !currentlyDisabled) {
			this.setCameraThrottle(this.config.ENABLED_CAMERA_THROTTLE);
		} else if (!lastDisabled && currentlyDisabled) {
			this.setCameraThrottle(this.config.DISABLED_CAMERA_THROTTLE);
		}
		lastDisabled = currentlyDisabled;

		if (!this.useThread) {
			visionIO.refreshData();
			visionIO.updateInputs(visionInputs);
		}
		this.visionEstimator.processCameraEstimates(this.getCameraInputsSnapshot());
	}

	public void logTelemetry() {
		int numEstimates = this.visionEstimator.getLatestProcessedCameraEstimates()
				.stream().filter(Optional::isPresent).toList().size();

		Logger.recordOutput(this.getKey() + "/numProcessedEstimates", numEstimates);
		Logger.recordOutput(this.getKey() + "/isMg2Enabled", this.visionEstimator.isMg2Enabled());
		Optional<VisionFieldPoseEstimate> lastEstimate = this.visionEstimator.getLastAcceptedEstimate();
		if (lastEstimate.isPresent()) {
			Logger.recordOutput(this.getKey() + "/lastEstimateTimestamp", lastEstimate.get().getTimestampSeconds());
			Logger.recordOutput(this.getKey() + "/fusedAccepted", lastEstimate.get().getVisionRobotPoseMeters());
		} else {
			Logger.recordOutput(this.getKey() + "/lastEstimateTimestamp", LoggerUtil.NO_DOUBLE);
			Logger.recordOutput(this.getKey() + "/fusedAccepted", LoggerUtil.NO_POSE_2D);
		}
	}

	public void logExtendedTelemetry() {
		this.logTelemetry();
		CameraIOInputs[] camsSnapshot = this.getCameraInputsSnapshot();
		List<Pose3d> targets = new ArrayList<>();
		Set<Integer> ids = new HashSet<>();
		// We have to log it like this because Logger.recordOutput doesn't accept
		// objects that hold arrays
		for (int i = 0; i < camsSnapshot.length; i++) {
			String prefix = this.getCameraKey(i);
			CameraIOInputs camInputs = camsSnapshot[i];
			MegatagPoseEstimate mt1Estimate = camInputs.megatag1PoseEstimate;
			MegatagPoseEstimate mt2Estimate = camInputs.megatag2PoseEstimate;
			Logger.recordOutput(prefix + "/seesTarget", camInputs.seesTarget);
			MegatagPoseEstimate.log(mt1Estimate, prefix + "/megatag1Estimate");
			MegatagPoseEstimate.log(mt2Estimate, prefix + "/megatag2Estimate");

			if (camInputs.seesTarget) {
				if (mt1Estimate != null) {
					for (int j = 0; j < mt1Estimate.fiducialIds.length; j++) {
						int id = mt1Estimate.fiducialIds[j];
						if (!ids.contains(id)) {
							ids.add(id);
							this.config.aprilTagFieldLayout.getTagPose(id).ifPresent(pose -> {
								targets.add(pose);
							});
						}

					}
				}

				if (mt2Estimate != null) {
					for (int j = 0; j < mt2Estimate.fiducialIds.length; j++) {
						int id = mt2Estimate.fiducialIds[j];
						if (!ids.contains(id)) {
							ids.add(id);
							this.config.aprilTagFieldLayout.getTagPose(id).ifPresent(pose -> {
								targets.add(pose);
							});
						}
					}
				}
			}

			Optional<VisionFieldPoseEstimate> estimate = this.visionEstimator.getLatestProcessedCameraEstimates()
					.get(i);
			if (estimate.isPresent()) {
				VisionFieldPoseEstimate est = estimate.get();
				Logger.recordOutput(prefix + "/processedEstimate/pose", est.getVisionRobotPoseMeters());
				Logger.recordOutput(prefix + "/processedEstimate/timestamp", est.getTimestampSeconds());
				Logger.recordOutput(prefix + "/processedEstimate/type", est.getEstimationMethod().name());
			} else {
				Logger.recordOutput(prefix + "/processedEstimate/pose", LoggerUtil.NO_POSE_2D);
				Logger.recordOutput(prefix + "/processedEstimate/timestamp", LoggerUtil.NO_DOUBLE);
				Logger.recordOutput(prefix + "/processedEstimate/type", "None");
			}
		}
		Logger.recordOutput(this.getKey() + "/VisibleTargets", targets.toArray(new Pose3d[0]));
	}

	public String getKey() {
		return "Subsystems/Vision";
	}

	public String getCameraKey(int cameraIndex) {
		return this.getKey() + "/Camera " + this.config.cameraConfigurations[cameraIndex].getName();
	}
}