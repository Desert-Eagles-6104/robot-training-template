package frc.DELib26.subsystems.vision.IOs;

import frc.DELib26.util.vision.CameraConfiguration;

import java.util.function.Supplier;

public class CameraIOSimPhoton extends CameraIOHardwareLimelight {

	public CameraIOSimPhoton(CameraConfiguration cameraConfig, Supplier<Double> robotYawSupplier) {
		super(cameraConfig, robotYawSupplier);
	}

	@Override
	public void crop(double xMin, double xMax, double yMin, double yMax) {
		// Not implemented in simulation
	}

}
