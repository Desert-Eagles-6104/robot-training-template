package frc.DELib26.subsystems.objectDetection.util;

import frc.DELib26.util.vision.CameraConfiguration;
import org.photonvision.estimation.TargetModel;

public class ObjectDetectionSubsystemConfiguration {
	public CameraConfiguration cameraConfiguration;
	public TargetModel simTargetModel;
	public String gamePieceName;

	/**
	 * Average pixel error to simulate in vision measurements (pixels).
	 */
	public double simAvgErrorPx = 0.35;
	/**
	 * Standard deviation of pixel error to simulate in vision measurements
	 * (pixels).
	 */
	public double simErrorStdDevPx = 0.5;

	public double simAvgLatencyMs = 30;
	public double simBaseStdDevXY = 0.2;
	public double simBaseStdDevYawDeg = 5.0;

	// run Photon sim at 10Hz (50Hz robot loop / 5)
	public final int SIM_UPDATE_DIV = 5;

	// cap number of simulated targets added to Photon
	public final int SIM_MAX_TARGETS = 6;

	// pose hashing tolerance (avoid rebuilds on tiny noise)
	public final double SIM_POS_EPS_M = 0.02;// 2cm
	public final double SIM_ROT_EPS_RAD = 0.03;// 1.7 deg
}
