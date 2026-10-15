package frc.DELib26.util;


public class Conversions {
	public static final double MILI_TO_SEC = 0.001;
	public static final double ANGLE_PER_ROTATION = 360.0;

	public static final double RADIANS_PER_ROTATION = 2.0 * Math.PI;

	/**
	 * Converts motor rotations to linear travel along a drum/wheel.
	 *
	 * <p><b>Returns meters per motor rotation.</b></p>
	 *
	 * @param gearing Mechanism rotations per motor rotation
	 *                (i.e., 1.0 / sensorToMechanismRatio).
	 * @param drumRadiusMeters Drum/wheel radius in meters.
	 * @return Linear distance (meters) per motor rotation.
	 */
	public static double linearDistanceMetersPerRotation(double gearing, double drumRadiusMeters) {
		return gearing * drumRadiusMeters * RADIANS_PER_ROTATION;
	}

	public static double rotationsPerLinearDistance(double gearing, double drumRadiusMeters) {
		return 1.0 / linearDistanceMetersPerRotation(gearing, drumRadiusMeters);
	}

	/**
	 * Convert wheel rotational speed into linear wheel speed.
	 *
	 * @param wheelRotPerSec Wheel velocity (rotations per second).
	 * @param circumference Wheel circumference (meters).
	 * @return Wheel linear velocity (meters per second).
	 */
	public static double RotPerSecToMPS(double wheelRotPerSec, double circumference) {
		return wheelRotPerSec * circumference;
	}

	/**
	 * Convert linear wheel speed into wheel rotational speed.
	 *
	 * @param wheelMPS Wheel linear velocity (meters per second).
	 * @param circumference Wheel circumference (meters).
	 * @return Wheel velocity (rotations per second).
	 * @throws IllegalArgumentException if circumference is zero.
	 */
	public static double MPSToRotPerSec(double wheelMPS, double circumference) {
		return wheelMPS / circumference;
	}

	/**
	 * Convert linear speed at a module into angular speed of the robot.
	 *
	 * @param wheelMPS Wheel linear velocity (m/s).
	 * @param radiusMeters Distance from robot center to module (m).
	 * @return Robot angular velocity (radians per second).
	 */
	public static double MPSToRadPerSec(double wheelMPS, double radiusMeters) {
		return wheelMPS / radiusMeters;
	}

	/**
	 * Convert angular robot speed into the linear speed a module must travel.
	 *
	 * @param radPerSec Robot angular velocity (rad/s).
	 * @param radiusMeters Distance from robot center to module (m).
	 * @return Module linear velocity (m/s).
	 */
	public static double RadPerSecToMPS(double radPerSec, double radiusMeters) {
		return radPerSec * radiusMeters;
	}

	/**
	 * Convert wheel position into linear distance traveled by the wheel.
	 *
	 * @param wheelRotations Wheel Position: (in Rotations)
	 * @param circumference Wheel Circumference: (in Meters)
	 * @return Wheel Distance: (in Meters)
	 */
	public static double rotationsToMeters(double wheelRotations, double circumference) {
		double wheelMeters = wheelRotations * circumference;
		return wheelMeters;
	}

	public static double degreesToRadians(double degrees) {
		double Radians = Math.toRadians(degrees);
		return Radians;
	}

	/**
	 * Convert linear distance traveled by the wheel into wheel position.
	 *
	 * @param wheelMeters Wheel Distance: (in Meters)
	 * @param circumference Wheel Circumference: (in Meters)
	 * @return Wheel Position: (in Rotations)
	 */
	public static double metersToRotations(double wheelMeters, double circumference) {
		double wheelRotations = wheelMeters / circumference;
		return wheelRotations;
	}

	/**
	 * Convert time in milliseconds to time in seconds.
	 *
	 * @param milliSeconds Time: (in Seconds)
	 * @return Time: (in seconds)
	 */
	public static double millToSeconds(double milliSeconds) {
		double seconds = milliSeconds / 1000.0;
		return seconds;
	}

	/**
	 * Convert time in seconds to time in milliseconds.
	 *
	 * @param ticksPerSecond Ticks per second: (in Ticks per Second)
	 * @return Time: (in seconds)
	 */
	public static double ticksToSeconds(double ticks, double ticksPerSecond) {
		double seconds = ticks / ticksPerSecond;
		return seconds;
	}

	/**
	 * Wraps a value to be within the range [0, max).
	 *
	 * @param value The value to wrap.
	 * @param max The maximum bound of the range (exclusive).
	 * @return The wrapped value within the range [0, max).
	 */
	public static double wrapToFromZero(double value, double max) {
		return wrapTo(value, 0, max);
	}

	/**
	 * Wraps a value to be within a specified range [min, max).
	 *
	 * @param value The value to wrap.
	 * @param min The minimum bound of the range (inclusive).
	 * @param max The maximum bound of the range (exclusive).
	 * @return The wrapped value within the range [min, max).
	 */
	public static double wrapTo(double value, double min, double max) {
		double range = max - min;
		if (!(range > 0.0)) {
			throw new IllegalArgumentException("min must be < max");
		}
		double wrapped = (value - min) % range;
		if (wrapped < 0) {
			wrapped += range;
		}
		return wrapped + min;
	}

	/**
	 * Wraps an integer value to be within a specified range [min, max).
	 *
	 * @param value The value to wrap.
	 * @param min   The minimum bound of the range (inclusive).
	 * @param max   The maximum bound of the range (exclusive).
	 * @return The wrapped value within the range [min, max).
	 */
	public static int wrapTo(int value, int min, int max) {
		int range = max - min;
		if (!(range > 0)) {
			throw new IllegalArgumentException("min must be < max");
		}
		int wrapped = (value - min) % range;
		if (wrapped < 0) {
			wrapped += range;
		}
		return wrapped + min;
	}

	/**
	 * Wraps an angle in degrees to the range [-180, 180).
	 * @param angleDegrees The angle in degrees to wrap.
	 * @return The wrapped angle within [-180, 180).
	 */
	public static double wrapTo180Degrees(double angleDegrees) {
		return wrapTo(angleDegrees, -180.0, 180.0);
	}

	/**
	 * Wraps an angle in degrees to the range [0, 360).
	 *
	 * @param angleDegrees The angle in degrees to wrap.
	 * @return The wrapped angle within [0, 360).
	 */
	public static double wrapTo360Degrees(double angleDegrees) {
		return wrapTo(angleDegrees, 0.0, 360.0);
	}

	/**
	 * Calculates the diagonal field of view (FOV) of a camera given its horizontal and vertical FOVs.
	 * @param horizontalFOVDegree camera horizontal FOV in degrees.
	 * @param verticalFOVDegrees camera vertical FOV in degrees.
	 * @return The diagonal FOV in degrees.
	 */
	public static double calculateCameraDiagonalFOV(double horizontalFOVDegree, double verticalFOVDegrees) {
		double h = Math.toRadians(horizontalFOVDegree);
		double v = Math.toRadians(verticalFOVDegrees);

		// diagonal half-angle obeys: tan(D/2)^2 = tan(H/2)^2 + tan(V/2)^2
		double tanH2 = Math.tan(h / 2.0);
		double tanV2 = Math.tan(v / 2.0);
		double d = 2.0 * Math.atan(Math.sqrt(tanH2 * tanH2 + tanV2 * tanV2));

		return Math.toDegrees(d);
	}

}
