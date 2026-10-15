package frc.DELib26.subsystems.objectDetection;

import frc.DELib26.subsystems.objectDetection.IOs.ObjectDetectionIO;
import frc.DELib26.subsystems.objectDetection.IOs.ObjectDetectionIO.ObjectDetectionIOInputs;
import frc.DELib26.subsystems.objectDetection.IOs.ObjectDetectionIOHardwareLimelight;
import frc.DELib26.subsystems.objectDetection.IOs.ObjectDetectionIOSimPhoton;
import frc.DELib26.subsystems.objectDetection.util.DetectedObject;
import frc.DELib26.subsystems.objectDetection.util.ObjectDetectionFrame;
import frc.DELib26.subsystems.objectDetection.util.ObjectDetectionSubsystemConfiguration;
import frc.DELib26.util.LoggerUtil;
import frc.DELib26.util.RobotState;
import frc.DELib26.util.RunMode;
import frc.DELib26.util.SubsystemDataProcessor;
import frc.DELib26.util.VirtualSubsystem;
import org.littletonrobotics.junction.Logger;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;
import java.util.Optional;

public class ObjectDetectionSubsystem extends VirtualSubsystem {

	private ObjectDetectionIO io;
	private ObjectDetectionIOInputs inputs;

	private final Object objectDetectionIOLock = new Object();
	private final boolean useThread;

	private final List<DetectedObject> detectedObjects = new ArrayList<>();

	public ObjectDetectionSubsystem(ObjectDetectionSubsystemConfiguration config, RobotState robotState) {
		if (RunMode.isSim()) {
			io = new ObjectDetectionIOSimPhoton(config, () -> robotState.getLatestRobotPose());
		} else {
			io = new ObjectDetectionIOHardwareLimelight(config.cameraConfiguration);
		}
		this.inputs = new ObjectDetectionIOInputs();

		this.useThread = true;
		if (this.useThread) {
			SubsystemDataProcessor.createAndStartSubsystemDataProcessor(
				() -> {
					synchronized (objectDetectionIOLock) {
						io.updateInputs(inputs);
					}
				},
				io,
				"ObjectDetectionIO"
			);
		}
	}

	private ObjectDetectionIOInputs getObjectDetectionsInputsSnapshot() {
		if (this.useThread) {
			synchronized (objectDetectionIOLock) {
				return this.inputs.copy();
			}
		} else {
			return this.inputs;
		}
	}

	@Override
	public void periodic() {
		if (!this.useThread) {
			io.refreshData();
			io.updateInputs(inputs);
		}

		ObjectDetectionIOInputs snapshot = this.getObjectDetectionsInputsSnapshot();

		detectedObjects.clear();
		if (snapshot.latestFrame != null && snapshot.latestFrame.hasTargets && snapshot.latestFrame.objects != null) {
			detectedObjects.addAll(Arrays.asList(snapshot.latestFrame.objects));
		}

	}

	public void logTelemetry() {
		final String base = getKey();
		final ObjectDetectionIOInputs snapshot = getObjectDetectionsInputsSnapshot();
		final ObjectDetectionFrame frame = snapshot.latestFrame;


		Logger.recordOutput(base + "/lastUpdateTimestampSec", snapshot.lastUpdateTimestampSec);

		final boolean hasFrame = (frame != null);
		Logger.recordOutput(base + "/frame/valid", hasFrame);

		final DetectedObject[] objs = (hasFrame && frame.objects != null) ? frame.objects : new DetectedObject[0];
		final int count = objs.length;

		Logger.recordOutput(base + "/frame/timestampSec", hasFrame ? frame.captureTimestampSec : LoggerUtil.NO_DOUBLE);
		Logger.recordOutput(base + "/frame/hasTargets", hasFrame && frame.hasTargets);
		Logger.recordOutput(base + "/frame/objectCount", count);

		Logger.recordOutput(base + "/frame/objects", frame.objects);

	}

	public void logExtendedTelemetry() {
		this.logTelemetry();
	}

	public Optional<List<DetectedObject>> getDetectedObjects() {
		if (this.detectedObjects.isEmpty()) {
			return Optional.empty();
		} else {
			return Optional.of(this.detectedObjects);
		}
	}

	public String getKey() {
		return "Subsystems/ObjectDetection";
	}

}
