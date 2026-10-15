package frc.robot.constants;

import frc.DELib26.util.BaseRobotConstants;

import java.util.Optional;

public class Constants extends BaseRobotConstants {

	public AutonomousConstants autonomousConstants;

	public Optional<SimulationConstants> simulationConstants = Optional.empty();

	public static class AutonomousConstants {
		public double MAX_START_TRANS_ERROR_METERS;
		public double MAX_START_HEADING_ERROR_DEGREES;
		public double AUTONOMOUS_VISION_LOOKBACK_TIME_SECONDS;
	}

	public static class SimulationConstants {}

}