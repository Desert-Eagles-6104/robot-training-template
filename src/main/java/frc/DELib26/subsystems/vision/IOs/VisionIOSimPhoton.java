package frc.DELib26.subsystems.vision.IOs;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Timer;
import frc.DELib26.subsystems.vision.visionUtil.VisionSubsystemConfiguration;
import frc.DELib26.util.Conversions;
import frc.DELib26.util.vision.CameraConfiguration;
import org.photonvision.PhotonCamera;
import org.photonvision.simulation.PhotonCameraSim;
import org.photonvision.simulation.SimCameraProperties;
import org.photonvision.simulation.VisionSystemSim;
import org.photonvision.targeting.PhotonPipelineResult;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;
import java.util.Optional;
import java.util.function.Supplier;

import static edu.wpi.first.units.Units.Degree;

public class VisionIOSimPhoton extends VisionIOHardwareLimelight {
	private final VisionSystemSim visionSim;
	private final Supplier<Pose2d> robotPoseSupplier;
	private final PhotonCamera[] cameras;
	private PhotonCameraSim[] cameraSims;
	private final NetworkTable[] tables;


	public VisionIOSimPhoton(CameraIO[] cameraIOs, VisionSubsystemConfiguration visionConfig, Supplier<Pose2d> robotPoseSupplier) {
		super(cameraIOs, visionConfig);
		this.robotPoseSupplier = robotPoseSupplier;

		this.visionSim = new VisionSystemSim("main");
		this.visionSim.addAprilTags(visionConfig.aprilTagFieldLayout);

		this.cameras = new PhotonCamera[cameraIOs.length];
		this.cameraSims = new PhotonCameraSim[cameraIOs.length];
		this.tables = new NetworkTable[cameraIOs.length];

		for (int i = 0; i < cameraIOs.length; i++) {
			CameraConfiguration cameraConfig = visionConfig.cameraConfigurations[i];
			this.cameras[i] = new PhotonCamera(cameraConfig.getName());
			this.tables[i] = NetworkTableInstance.getDefault().getTable(cameraConfig.limelightTableName);

			double cameraFovDiagnonalDegrees = Conversions.calculateCameraDiagonalFOV(
				cameraConfig.cameraModel.horizontalFOVDegrees,
				cameraConfig.cameraModel.verticalFOVDegrees
			);

			SimCameraProperties prop = new SimCameraProperties();
			prop.setCalibration(
				cameraConfig.cameraModel.maxWidthPx,
				cameraConfig.cameraModel.maxHeightPx,
				Rotation2d.fromDegrees(cameraFovDiagnonalDegrees)
			);

			prop.setCalibError(visionConfig.simAvgErrorPx, visionConfig.simErrorStdDevPx);
			prop.setFPS(45);
			prop.setAvgLatencyMs(visionConfig.simAvgLatencyMs);
			prop.setLatencyStdDevMs(5);
			prop.setExposureTimeMs(0.65);

			this.cameraSims[i] = new PhotonCameraSim(this.cameras[i], prop);
			this.cameraSims[i].setMinTargetAreaPixels(1000);
			this.cameraSims[i].enableRawStream(true);
			this.cameraSims[i].enableProcessedStream(true);
			this.cameraSims[i].enableDrawWireframe(true);

			visionSim.addCamera(this.cameraSims[i], cameraConfig.getRobotToCamera());

		}

	}

	@Override
	public void refreshData() {
		Pose2d estimatedPose = this.robotPoseSupplier.get();
		if (estimatedPose != null) {
			visionSim.update(estimatedPose);
		}

		for (int i = 0; i < cameras.length; i++) {
			writeToTable(cameras[i].getAllUnreadResults(), this.tables[i], cameraSims[i]);
		}
		super.refreshData();
	}

	/** Generates robot pose data from PhotonVision results. */
	private List<Double> getBotpose(
			Transform3d fieldToCamera,
			int numTags,
			PhotonPipelineResult result,
			PhotonCameraSim cameraSim) {
		if (result == null || result.targets.isEmpty()) return null;
		//it works in fpga time
		Optional<Transform3d> optRobotToCamera = visionSim.getRobotToCamera(cameraSim, Timer.getFPGATimestamp());

		Pose3d fieldToRobot;
		if (optRobotToCamera.isPresent()) {
			Transform3d cameraToRobot = optRobotToCamera.get().inverse();
			Pose3d robotPose3d =
					new Pose3d(fieldToCamera.getTranslation(), fieldToCamera.getRotation())
							.transformBy(cameraToRobot);
			fieldToRobot = robotPose3d;
		} else {
			fieldToRobot = new Pose3d(fieldToCamera.getTranslation(), fieldToCamera.getRotation());
		}

		List<Double> pose_data = new ArrayList<>(
						Arrays.asList(
								fieldToRobot.getX(),
								fieldToRobot.getY(),
								fieldToRobot.getZ(),
								0.0,
								0.0,
								fieldToRobot.getRotation().getMeasureZ().in(Degree),
								result.metadata.getLatencyMillis(),
								(double) numTags,
								0.0,
								0.0,
								result.getBestTarget().getArea()
							)
						);

		for (var target : result.targets) {
			pose_data.addAll(
					Arrays.asList(
						(double) target.getFiducialId(),
						target.getYaw(), // txnc
						target.getPitch(), // tync
						target.area, // ta
						0.0, // distToCamera
						0.0, // distToRobot
						target.getPoseAmbiguity() // ambiguity
					)
				);
		}
		return pose_data;
	}

	/**
	 * Writes simulated vision data to NetworkTables for consumption by Limelight processing code.
	 */
	private void writeToTable(List<PhotonPipelineResult> results, NetworkTable table, PhotonCameraSim cameraSim) {
		boolean seesTarget = false;
		for (var result : results) {
			List<Double> pose_data = null;
			if (result.getMultiTagResult().isPresent()) {
				var multiTagResult = result.getMultiTagResult().get();
				Transform3d best = multiTagResult.estimatedPose.best;

				pose_data = getBotpose(best, multiTagResult.fiducialIDsUsed.size(), result, cameraSim);
			} else if (result.hasTargets()) {
				var bestTarget = result.getBestTarget();
				Transform3d best = this.visionConfig.aprilTagFieldLayout
									.getTagPose(bestTarget.getFiducialId())
									.get()
									.minus(Pose3d.kZero)
									.plus(bestTarget.bestCameraToTarget.inverse());

				pose_data = getBotpose(best, 1, result, cameraSim);
			}

			if (pose_data != null) {
				table.getEntry("botpose_wpiblue").setDoubleArray(pose_data.stream().mapToDouble(Double::doubleValue).toArray());
				table.getEntry("botpose_orb_wpiblue").setDoubleArray(pose_data.stream().mapToDouble(Double::doubleValue).toArray());

				double[] rawFiducials = result.targets.stream()
					.flatMapToDouble(t -> java.util.stream.DoubleStream.of(
						(double) t.getFiducialId(),
						t.getYaw(),
						t.getPitch(),
						t.getArea(),
						0.0,
						0.0,
						t.getPoseAmbiguity()
					))
					.toArray();

				table.getEntry("rawfiducials").setDoubleArray(rawFiducials);
				table.getEntry("tid").setDouble(result.getBestTarget().getFiducialId());

				double[] mt1Std = this.computeMegatagStdDevsForCamera(result, visionConfig, 1.0);
				double[] mt2Std = this.computeMegatagStdDevsForCamera(result, visionConfig, 0.7);

				double[] stddevs = new double[12];
				System.arraycopy(mt1Std, 0, stddevs, 0, 6);
				System.arraycopy(mt2Std, 0, stddevs, 6, 6);
				table.getEntry("stddevs").setDoubleArray(stddevs);

				seesTarget = true;
			}

			table.getEntry("cl").setDouble(result.metadata.getLatencyMillis());
		}
		table.getEntry("tv").setInteger(seesTarget ? 1 : 0);
	}

	private double[] computeMegatagStdDevsForCamera(
		PhotonPipelineResult result,
		VisionSubsystemConfiguration visionConfig,
		double megatagScale
	) {
		var bestTarget = result.getBestTarget();

		int numTags = result.getMultiTagResult()
			.map(m -> Math.max(1, m.fiducialIDsUsed.size()))
			.orElse(1);

		double distance = bestTarget.getBestCameraToTarget()
			.getTranslation()
			.getNorm();

		double ambiguity = bestTarget.getPoseAmbiguity();

		// base noise (meters / degrees)
		double baseXY = visionConfig.simBaseStdDevXY;
		double baseTheta = visionConfig.simBaseStdDevYawDeg;

		double tagScale = 1.0 / Math.sqrt(numTags);
		double distScale = 1.0 + 0.4 * Math.max(0.0, distance - 1.0);
		double ambScale = 1.0 + 2.0 * ambiguity;

		double scale = tagScale * distScale * ambScale * megatagScale;

		double xyStd = baseXY * scale;
		double thetaStdDeg = baseTheta * scale;

		return new double[] { xyStd, xyStd, 0.0, 0.0, 0.0, thetaStdDeg };
	}

}