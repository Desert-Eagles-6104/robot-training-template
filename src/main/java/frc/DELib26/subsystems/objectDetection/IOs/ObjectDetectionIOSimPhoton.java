package frc.DELib26.subsystems.objectDetection.IOs;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Timer;
import frc.DELib26.subsystems.objectDetection.util.ObjectDetectionSubsystemConfiguration;
import frc.DELib26.util.Conversions;
import org.ironmaple.simulation.SimulatedArena;
import org.photonvision.PhotonCamera;
import org.photonvision.estimation.TargetModel;
import org.photonvision.simulation.PhotonCameraSim;
import org.photonvision.simulation.SimCameraProperties;
import org.photonvision.simulation.VisionSystemSim;
import org.photonvision.simulation.VisionTargetSim;
import org.photonvision.targeting.PhotonPipelineResult;

import java.util.Arrays;
import java.util.List;
import java.util.function.Supplier;

public class ObjectDetectionIOSimPhoton extends ObjectDetectionIOHardwareLimelight {
	private final ObjectDetectionSubsystemConfiguration config;
	private final VisionSystemSim visionSim;
	private final Supplier<Pose2d> robotPoseSupplier;

	private final NetworkTable table;

	private final PhotonCamera camera;
	private final PhotonCameraSim cameraSim;

	private final TargetModel model;
	private final String gamePieceType;

	private int tick = 0;
	private long lastPiecesHash = 0L;
	private Pose3d[] lastPieces = new Pose3d[0];

	// hold last published values so NT doesn’t flap
	private boolean lastHasTarget = false;
	private double lastTx = 0.0, lastTy = 0.0, lastTa = 0.0;
	private double lastLatencyMs = 0.0;
	private long lastTid = -1;
	private double lastUpdateTs = 0.0;

	public ObjectDetectionIOSimPhoton(
		ObjectDetectionSubsystemConfiguration config,
			Supplier<Pose2d> robotPoseSupplier
	) {
		super(config.cameraConfiguration);
		this.config = config;

		this.robotPoseSupplier = robotPoseSupplier;
		this.gamePieceType = config.gamePieceName;
		this.model = config.simTargetModel;

		this.table = NetworkTableInstance.getDefault().getTable(cameraConfig.limelightTableName);

		this.visionSim = new VisionSystemSim("objectDetection");

		this.camera = new PhotonCamera(cameraConfig.getName());

		double cameraFovDiagDeg = Conversions.calculateCameraDiagonalFOV(
				cameraConfig.cameraModel.horizontalFOVDegrees,
				cameraConfig.cameraModel.verticalFOVDegrees
		);

		SimCameraProperties prop = new SimCameraProperties();
		prop.setCalibration(
			cameraConfig.cameraModel.maxWidthPx,
			cameraConfig.cameraModel.maxHeightPx,
			Rotation2d.fromDegrees(cameraFovDiagDeg)
		);
		prop.setCalibError(
			config.simAvgErrorPx,
			config.simErrorStdDevPx
		);
		prop.setFPS(30);// keep it realistic + cheaper than 45
		prop.setLatencyStdDevMs(5);
		prop.setExposureTimeMs(0.65);

		this.cameraSim = new PhotonCameraSim(camera, prop);
		this.cameraSim.setMinTargetAreaPixels(2500);
		this.cameraSim.enableRawStream(true);
		this.cameraSim.enableProcessedStream(true);
		this.cameraSim.enableDrawWireframe(true);

		// add camera to sim
		this.visionSim.addCamera(this.cameraSim, cameraConfig.getRobotToCamera());

		// initial publish (no target)
		publishHeldValues();
	}

	@Override
	public void refreshData() {
		// Always refresh your LL consumer after we publish / hold values
		// but only run heavy sim at reduced rate.
		boolean doHeavyUpdate = ((tick++ % this.config.SIM_UPDATE_DIV) == 0);

		if (doHeavyUpdate) {
			// 1) Update target set ONLY if pieces meaningfully changed
			Pose3d[] pieces = SimulatedArena.getInstance().getGamePiecesArrayByType(gamePieceType);

			Pose3d[] capped = capPieces(pieces, this.config.SIM_MAX_TARGETS);

			long h = hashPieces(capped, this.config.SIM_POS_EPS_M, this.config.SIM_ROT_EPS_RAD);

			if (h != lastPiecesHash) {
				lastPiecesHash = h;
				lastPieces = capped;
				rebuildTargets(lastPieces);
			}

			// 2) Run sim update (heavy)
			Pose2d robotPose2d = robotPoseSupplier.get();
			if (robotPose2d == null) robotPose2d = new Pose2d();

			visionSim.update(robotPose2d);

			// 3) Consume results from PhotonCamera NT buffer (cheap)
			// NOTE: we don’t need results from every unread frame, just latest with target
			var unread = camera.getAllUnreadResults();
			consumeAndHoldBest(unread);

			lastUpdateTs = Timer.getFPGATimestamp();
			publishHeldValues();
		} else {
			// Between heavy updates, just re-publish held values (optional).
			// This keeps LL keys stable even if your consumer reads every tick.
			publishHeldValues();
		}

		super.refreshData();
	}

	private void consumeAndHoldBest(List<PhotonPipelineResult> results) {
		boolean has = false;
		double tx = lastTx, ty = lastTy, ta = lastTa;
		long tid = lastTid;

		// latency is in result.metadata, but for sim it’s not super critical
		double lat = lastLatencyMs;

		for (var r : results) {
			lat = r.metadata.getLatencyMillis();

			if (!r.hasTargets()) continue;

			var t = r.getBestTarget();
			has = true;
			tx = t.getYaw();
			ty = t.getPitch();
			ta = t.getArea();
			tid = t.getFiducialId();
		}

		lastHasTarget = has;
		lastTx = tx;
		lastTy = ty;
		lastTa = ta;
		lastTid = tid;
		lastLatencyMs = lat;
	}

	private void publishHeldValues() {
		table.getEntry("tv").setInteger(lastHasTarget ? 1 : 0);
		table.getEntry("tx").setDouble(lastTx);
		table.getEntry("ty").setDouble(lastTy);
		table.getEntry("ta").setDouble(lastTa);

		table.getEntry("tl").setDouble(lastLatencyMs);
		table.getEntry("cl").setDouble(lastLatencyMs);
		table.getEntry("tid").setInteger(lastTid);

		writeFakeRawDetections(table, lastHasTarget, lastTx, lastTy, lastTa);

		// optional debug
		table.getEntry("od_lastUpdate_s").setDouble(lastUpdateTs);
		table.getEntry("od_targetCount").setInteger(lastPieces.length);
	}

	private void rebuildTargets(Pose3d[] pieces) {
		// remove only the target set we own
		visionSim.removeVisionTargets(this.gamePieceType);

		for (Pose3d p : pieces) {
			visionSim.addVisionTargets(this.gamePieceType, new VisionTargetSim(p, model));
		}
	}

	private static Pose3d[] capPieces(Pose3d[] pieces, int max) {
		if (pieces == null || pieces.length == 0) return new Pose3d[0];
		int n = Math.min(pieces.length, max);
		return Arrays.copyOfRange(pieces, 0, n);
	}

	private static long hashPieces(Pose3d[] poses, double posEpsM, double rotEpsRad) {
		long h = 1469598103934665603L;
		for (Pose3d p : poses) {
			long x = Math.round(p.getX() / posEpsM);
			long y = Math.round(p.getY() / posEpsM);
			long z = Math.round(p.getZ() / posEpsM);
			long yaw = Math.round(p.getRotation().getZ() / rotEpsRad);

			h ^= x;   h *= 1099511628211L;
			h ^= y;   h *= 1099511628211L;
			h ^= z;   h *= 1099511628211L;
			h ^= yaw; h *= 1099511628211L;
		}
		return h;
	}

	private void writeFakeRawDetections(
			NetworkTable table,
			boolean hasTarget,
			double txDeg,
			double tyDeg,
			double areaPercent
		) {

		if (!hasTarget) {
			table.getEntry("rawdetections").setDoubleArray(new double[0]);
			return;
		}

		double txnc = clamp(txDeg / 30.0, -1.0, 1.0);
		double tync = clamp(tyDeg / 20.0, -1.0, 1.0);

		int classId = 0;
		double ta = areaPercent;

		double halfW = 0.10;
		double halfH = 0.07;

		double c0x = txnc - halfW, c0y = tync - halfH;
		double c1x = txnc + halfW, c1y = tync - halfH;
		double c2x = txnc + halfW, c2y = tync + halfH;
		double c3x = txnc - halfW, c3y = tync + halfH;

		double[] arr = new double[] {
				classId, txnc, tync, ta,
				c0x, c0y, c1x, c1y, c2x, c2y, c3x, c3y
		};

		table.getEntry("rawdetections").setDoubleArray(arr);
	}

	private static double clamp(double v, double lo, double hi) {
		return Math.max(lo, Math.min(hi, v));
	}
}