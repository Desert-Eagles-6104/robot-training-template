package frc.DELib26.subsystems.objectDetection.util;

public final class ObjectDetectionFrame {
	public String cameraName;
	public boolean hasTargets;
	public double captureTimestampSec;
	public DetectedObject[] objects = new DetectedObject[0];
}
