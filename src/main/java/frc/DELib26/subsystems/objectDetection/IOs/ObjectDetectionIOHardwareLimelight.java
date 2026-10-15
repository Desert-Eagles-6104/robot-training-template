package frc.DELib26.subsystems.objectDetection.IOs;

import edu.wpi.first.wpilibj.Timer;
import frc.DELib26.subsystems.objectDetection.util.DetectedObject;
import frc.DELib26.subsystems.objectDetection.util.ObjectDetectionFrame;
import frc.DELib26.util.vision.CameraConfiguration;
import frc.DELib26.util.vision.LimelightHelpers;

public class ObjectDetectionIOHardwareLimelight implements ObjectDetectionIO {

	protected final CameraConfiguration cameraConfig;

	public ObjectDetectionIOHardwareLimelight(CameraConfiguration cameraConfig) {
		this.cameraConfig = cameraConfig;
		LimelightHelpers.setCameraPose_RobotSpace(
			cameraConfig.limelightTableName,
			cameraConfig.forward,
			cameraConfig.side,
			cameraConfig.up,
			cameraConfig.roll,
			cameraConfig.pitch,
			cameraConfig.yaw
		);
	}

	// Latency (ms) -> estimate capture timestamp in seconds
	private double nowSec;
	private double latencyMs;
	private double captureTimestampSec;
	// Raw detections (Neural Detector pipeline)
	private LimelightHelpers.RawDetection[] raw;

	@Override
	public void refreshData() {
		this.nowSec = Timer.getFPGATimestamp();

		// Latency (ms) -> estimate capture timestamp in seconds
		this.latencyMs =
				LimelightHelpers.getLatency_Pipeline(cameraConfig.limelightTableName)
						+ LimelightHelpers.getLatency_Capture(cameraConfig.limelightTableName);
		this.captureTimestampSec = this.nowSec - (latencyMs / 1000.0);

		this.raw = LimelightHelpers.getRawDetections(cameraConfig.limelightTableName);
	}

	@Override
	public void updateInputs(ObjectDetectionIOInputs inputs) {
		ObjectDetectionFrame frame = new ObjectDetectionFrame();
		frame.cameraName = cameraConfig.limelightTableName;
		frame.captureTimestampSec = captureTimestampSec;
		frame.hasTargets = raw.length > 0;

		DetectedObject[] objs = new DetectedObject[raw.length];
		for (int i = 0; i < raw.length; i++) {
			LimelightHelpers.RawDetection d = raw[i];

			DetectedObject o = new DetectedObject(
				d.txnc,
				d.tync,
				d.ta,
				captureTimestampSec
			);

			objs[i] = o;
		}

		frame.objects = objs;

		inputs.lastUpdateTimestampSec = this.nowSec;
		inputs.latestFrame = frame;

	}
}