package frc.robot;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.DELib26.subsystems.vision.visionUtil.VisionSubsystemConfiguration;
import frc.DELib26.util.RobotState;

public class RobotState2026 extends RobotState {

	private ChassisSpeeds latestFieldRelativeSpeeds;
	private boolean kickerHasFuel = false;
	private int fuelShotCount = 0;


	public RobotState2026(VisionSubsystemConfiguration visionConfig) {
		super(visionConfig);
	}

	public boolean kickerHasFuel() {
		return this.kickerHasFuel;
	}

	public void setKickerHasFuel(boolean newState) {
		this.kickerHasFuel = newState;
	}

	public int getFuelShotCount() {
		return fuelShotCount;
	}

	public void incrementFuelShotCount() {
		this.fuelShotCount++;
	}

	public void setLatestFieldRelativeSpeeds(ChassisSpeeds latestFieldRelativeSpeeds) {
		this.latestFieldRelativeSpeeds = latestFieldRelativeSpeeds;
	}

	public ChassisSpeeds getLatestFieldRelativeSpeeds() {
		return latestFieldRelativeSpeeds;
	}
}
