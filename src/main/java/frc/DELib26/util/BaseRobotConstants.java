package frc.DELib26.util;

import frc.DELib26.subsystems.objectDetection.util.ObjectDetectionSubsystemConfiguration;
import frc.DELib26.subsystems.swerve.swerveUtil.SwerveConstants;
import frc.DELib26.subsystems.vision.visionUtil.VisionSubsystemConfiguration;

import java.util.Optional;

public class BaseRobotConstants {

	public SwerveConstants swerveConstants;
	public VisionSubsystemConfiguration visionConfig;
	public Optional<ObjectDetectionSubsystemConfiguration> objectDetectionConfig = Optional.empty();
}
