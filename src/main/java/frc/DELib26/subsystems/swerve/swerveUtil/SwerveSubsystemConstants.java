package frc.DELib26.subsystems.swerve.swerveUtil;

import edu.wpi.first.math.geometry.Rotation2d;

public class SwerveSubsystemConstants {
	public static final double CONTROLLER_DEADBAND = 0.1;

	public static final double CHOREO_PATH_TRANSLATION_ERROR_TOLERANCE_METERS = 0.0125;
	public static final double DRIVE_TO_POINT_TRANSLATION_ERROR_TOLERANCE_METERS = 0.0256;
	public static final double DRIVE_TO_POINT_HEADING_ERROR_TOLERANCE_DEGREES = 5.0;
	public static final double DRIVE_TO_POINT_MIN_TRANSLATION_ERROR_FOR_FRICTION_METERS = 0.0125;
	public static final double DRIVE_TO_POINT_STATIC_FRICTION_CONSTANT = 0.02;
	public static final double DRIVE_TO_POINT_MIN_VELOCITY_FOR_ZERO_VALUE = 0.05;

	public static final Rotation2d ROTATION_ERROR_MARGIN_FOR_ROTATION_LOCK = Rotation2d.fromDegrees(3);
	public static final double ROTATION_LOCK_MIN_ANGULAR_VELOCITY_FOR_ZERO_VALUE_RAD_PER_SEC = 0.1;

	public static final double HEADING_STABILITY_DISTANCE_METERS = 0.1;

	/*
	 * "g" unit - deceleration threshold to detect collision
	 */
	public static final double FIELD_OF_MERIT_COLLISION_THRESHOLD_G = 2;
	/*
	 * smoothing for collision detection accelerometer readings
	 */
	public static final double FIELD_OF_MERIT_COLLISION_FILTER_ALPHA = 0.2;
	/*
	 * 2 cm/s absolute "noise" floor for skid detection
	 */
	public static final double FIELD_OF_MERIT_SKID_ABS_TRANS_EPS = 0.02;
	/*
	 * 5% of rotation speed - relative threshold for skid detection
	 */
	public static final double FIELD_OF_MERIT_SKID_REL_TRANS_EPS = 0.05;
	/*
	 * Skid ratio threshold for detecting skidding
	 */
	public static final double FIELD_OF_MERIT_SKID_RATIO_THRESHOLD = 1.5;
}
