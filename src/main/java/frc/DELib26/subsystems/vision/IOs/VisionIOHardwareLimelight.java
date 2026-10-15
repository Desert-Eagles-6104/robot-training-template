package frc.DELib26.subsystems.vision.IOs;

import frc.DELib26.subsystems.vision.visionUtil.VisionSubsystemConfiguration;

public class VisionIOHardwareLimelight implements VisionIO {

	protected final CameraIO[] cameraIOs;

	protected final VisionSubsystemConfiguration visionConfig;

	public VisionIOHardwareLimelight(CameraIO[] cameraIOs, VisionSubsystemConfiguration visionConfig) {
		this.cameraIOs = cameraIOs;
		this.visionConfig = visionConfig;
	}

	@Override
	public void refreshData() {
		for (CameraIO cameraIO : cameraIOs) {
			cameraIO.refreshData();
		}
	}

	@Override
	public void updateInputs(VisionIOInputs inputs) {
		for (int i = 0; i < cameraIOs.length; i++) {
			this.cameraIOs[i].updateInputs(inputs.cameraInputs[i]);
		}
	}

	@Override
	public void setCameraThrottle(int throttle) {
		for (CameraIO cameraIO : cameraIOs) {
			cameraIO.setThrottle(throttle);
		}
	}

}
