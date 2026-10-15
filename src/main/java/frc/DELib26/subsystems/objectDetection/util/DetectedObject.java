package frc.DELib26.subsystems.objectDetection.util;

public record DetectedObject(
	double yawDeg,
	double pitchDeg,
	double area,
	double captureTimestampSec
) {}
