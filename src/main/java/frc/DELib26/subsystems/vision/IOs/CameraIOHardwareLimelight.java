package frc.DELib26.subsystems.vision.IOs;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.wpilibj.DriverStation;
import frc.DELib26.subsystems.vision.visionUtil.MegatagPoseEstimate;
import frc.DELib26.subsystems.vision.visionUtil.VisionSubsystemConfiguration;
import frc.DELib26.util.vision.CameraConfiguration;
import frc.DELib26.util.vision.LimelightHelpers;
import frc.DELib26.util.vision.LimelightHelpers.PoseEstimate;

import java.util.Arrays;
import java.util.function.Supplier;

public class CameraIOHardwareLimelight implements CameraIO {
	// First AprilTag Limelight
	protected final CameraConfiguration cameraConfig;

	/**
	 * Whether the limelight has any valid targets (0 or 1)
	 */
	protected boolean seesTarget = false;

	private PoseEstimate mt1PoseEstimate;
	private PoseEstimate mt2PoseEstimate;
	private Pose3d megatag1Pose3d;
	private Pose3d megatag2Pose3d;
	private double[] standardDeviations;
	private double currentID = 0;

	private final Supplier<Double> robotYawSupplier;

	// protected double cropXMin = -1;
	// protected double cropXMax = 1;
	// protected double cropYMin = -1;
	// protected double cropYMax = 1;

	// protected double xFOV = 62.5;
	// protected double yFOV = 48.9;

	public CameraIOHardwareLimelight(CameraConfiguration cameraConfig, Supplier<Double> robotYawSupplier) {
		this.cameraConfig = cameraConfig;
		LimelightHelpers.setPipelineIndex(cameraConfig.limelightTableName, 0);
		LimelightHelpers.SetIMUMode(cameraConfig.limelightTableName, 0);
		LimelightHelpers.setCameraPose_RobotSpace(
			cameraConfig.limelightTableName,
			cameraConfig.forward,
			cameraConfig.side,
			cameraConfig.up,
			cameraConfig.roll,
			cameraConfig.pitch,
			cameraConfig.yaw
		);
		this.robotYawSupplier = robotYawSupplier;
	}

	private static boolean hasAnyFiducials(PoseEstimate est) {
		return est != null
			&& est.rawFiducials != null
			&& Arrays.stream(est.rawFiducials).anyMatch(java.util.Objects::nonNull);
	}

	@Override
	public void updateInputs(CameraIOInputs inputs) {
		inputs.seesTarget = this.seesTarget;
		inputs.megatag1PoseEstimate = null;
		inputs.megatag2PoseEstimate = null;

		if (LimelightHelpers.validPoseEstimate(this.mt1PoseEstimate) && hasAnyFiducials(this.mt1PoseEstimate)) {
			inputs.megatag1PoseEstimate = new MegatagPoseEstimate(
				this.seesTarget,
				this.mt1PoseEstimate,
				Arrays.copyOfRange(
					this.standardDeviations,
					0,
					VisionSubsystemConfiguration.PER_MEGATAG_STD_DEV_COUNT
				),
				this.megatag1Pose3d
			);
		}

		if (LimelightHelpers.validPoseEstimate(this.mt2PoseEstimate) && hasAnyFiducials(this.mt2PoseEstimate)) {
			inputs.megatag2PoseEstimate = new MegatagPoseEstimate(
				this.seesTarget,
				this.mt2PoseEstimate,
				Arrays.copyOfRange(
					this.standardDeviations,
					VisionSubsystemConfiguration.PER_MEGATAG_STD_DEV_COUNT,
					VisionSubsystemConfiguration.EXPECTED_STD_DEV_ARRAY_LENGTH
				),
				this.megatag2Pose3d
			);
		}

	}

	@Override
	public void refreshData() {
		this.seesTarget = LimelightHelpers.getTV(this.cameraConfig.limelightTableName);

		if (!this.seesTarget) {
			this.resetData();
			return;
		}

		try {
			LimelightHelpers.SetRobotOrientation(
				this.cameraConfig.limelightTableName,
				robotYawSupplier.get(),
				0.0,
				0.0,
				0.0,
				0.0,
				0.0
			);
			this.mt1PoseEstimate = LimelightHelpers.getBotPoseEstimate_wpiBlue(this.cameraConfig.limelightTableName);
			this.mt2PoseEstimate = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(this.cameraConfig.limelightTableName);


			double[] botPoseArr = LimelightHelpers.getBotPose_wpiBlue(this.cameraConfig.limelightTableName);
			this.megatag1Pose3d = (botPoseArr != null) ? LimelightHelpers.toPose3D(botPoseArr) : null;

			botPoseArr = LimelightHelpers.getBotPose_wpiBlue_MegaTag2(this.cameraConfig.limelightTableName);
			this.megatag2Pose3d = (botPoseArr != null) ? LimelightHelpers.toPose3D(botPoseArr) : null;

			double[] std = LimelightHelpers.getStandardDeviations(this.cameraConfig.limelightTableName);
			if (std == null || std.length != VisionSubsystemConfiguration.EXPECTED_STD_DEV_ARRAY_LENGTH) {
				DriverStation.reportWarning(
					"Limelight stddevs length mismatch. Expected: "
					+ VisionSubsystemConfiguration.EXPECTED_STD_DEV_ARRAY_LENGTH
					+ ", Actual: " + (std == null ? "null" : std.length), false);
				std = new double[VisionSubsystemConfiguration.EXPECTED_STD_DEV_ARRAY_LENGTH];
			}
			this.standardDeviations = std;

			this.currentID = LimelightHelpers.getFiducialID(this.cameraConfig.limelightTableName);
		} catch (Exception e) {
			DriverStation.reportError("Error processing Limelight data: " + e.getMessage(), e.getStackTrace());
			this.resetData();
		}
	}

	private void resetData() {
		this.mt1PoseEstimate = null;
		this.mt2PoseEstimate = null;
		this.megatag1Pose3d = null;
		this.megatag2Pose3d = null;
		this.standardDeviations = new double[VisionSubsystemConfiguration.EXPECTED_STD_DEV_ARRAY_LENGTH];
	}

	/**
	 * Returns the estimated robot pose based on the AprilTag camera's data.
	 * If no pose is estimated, it returns a default PoseEstimate with a zero
	 * Pose2d.
	 *
	 * @return PoseEstimate
	 */
	protected PoseEstimate getEstimatedRobotPose() {
		return this.mt1PoseEstimate;
	}

	/**
	 * Returns the estimated robot pose based on the AprilTag camera's data.
	 * If no pose is estimated, it returns null.
	 *
	 * @return PoseEstimate or null if no valid pose is available.
	 */
	protected PoseEstimate getEstimatedRobotPoseNoDefault() {
		return this.mt1PoseEstimate;
	}

	/**
	 * Crops the camera image to the specified window.
	 * The crop values are in the range of -1.0 to 1.0, where -1.0 is the left/top
	 * edge and 1.0 is the right/bottom edge.
	 *
	 * @param cropXMin Minimum horizontal crop value (-1.0 to 1.0)
	 * @param cropXMax Maximum horizontal crop value (-1.0 to 1.0)
	 * @param cropYMin Minimum vertical crop value (-1.0 to 1.0)
	 * @param cropYMax Maximum vertical crop value (-1.0 to 1.0)
	 */
	protected void crop(double cropXMin, double cropXMax, double cropYMin, double cropYMax) {
		LimelightHelpers.setCropWindow(this.cameraConfig.limelightTableName, cropXMin, cropXMax, cropYMin, cropYMax);
	}

	protected double getCurrentID() {
		return this.currentID;
	}

	@Override
	public void setThrottle(int throttle){
		LimelightHelpers.SetThrottle(this.cameraConfig.limelightTableName, throttle);
	}
}
