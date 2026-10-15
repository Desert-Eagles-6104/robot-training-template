package frc.DELib26.util;

import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Current;
import org.littletonrobotics.junction.LoggedRobot;

/**
 * A utility class that contains project-wide constants.
 */
public class ProjectConstants {
	public static final double GRAVITY = 9.80665;
	/**
	 * An integer error code used to signify an error state.
	 */
	public static final int ERROR_CODE_INT = -6104;

	/**
	 * A double error code used to signify an error state.
	 */
	public static final double ERROR_CODE_DOUBLE = -6104.6104;

	/**
	 * A very large variance used to down-weight or effectively reject unreliable
	 * measurements.
	 * <p>
	 * Unit: variance in the same units as the underlying state (e.g., m^2 or
	 * rad^2).
	 * </p>
	 */
	public static final double LARGE_VARIANCE = 1e6;

	/**
	 * A very small variance used to up-weight or effectively trust highly reliable
	 * measurements.
	 * <p>
	 * Unit: variance in the same units as the underlying state (e.g., m^2 or
	 * rad^2).
	 * </p>
	 */
	public static final double MIN_VARIANCE = 1e-6;


	/**
	 * The default period for the robot's main loop in seconds.
	 * This value is typically used to define the update rate of the robot's control system.
	 * Matches the default period defined in {@code LoggedRobot.defaultPeriodSecs}.
	 * {@link LoggedRobot#defaultPeriodSecs}
	 */
	public static final double DEFAULT_PERIOD_SECS = LoggedRobot.defaultPeriodSecs;

	public static final Current SIM_DIO_CURRENT = Units.Amps.of(0.05);

	public static final Current SIM_CAN_RANGE_CURRENT = Units.Amps.of(0.1);
}