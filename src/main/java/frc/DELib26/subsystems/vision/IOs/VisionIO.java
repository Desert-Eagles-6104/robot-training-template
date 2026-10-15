package frc.DELib26.subsystems.vision.IOs;

import frc.DELib26.subsystems.vision.IOs.CameraIO.CameraIOInputs;
import frc.DELib26.util.SubsystemDataProcessor;

public interface VisionIO extends SubsystemDataProcessor.IODataRefresher {

	public class VisionIOInputs {

		public CameraIOInputs[] cameraInputs;

		public VisionIOInputs(int numCameras) {
			this.cameraInputs = new CameraIOInputs[numCameras];
			for (int i = 0; i < numCameras; i++) {
				this.cameraInputs[i] = new CameraIOInputs();
			}
		}

	}

	public void updateInputs(VisionIOInputs inputs);

	public void setCameraThrottle(int throttle);

}
