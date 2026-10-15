package frc.DELib26.subsystems.vision.visionUtil;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import frc.DELib26.util.vision.CameraConfiguration;

import java.util.Optional;

public class VisionSubsystemConfiguration {

	// Standard deviation constants
	public static final int MEGATAG_X_STD_DEV_INDEX = 0;
	public static final int MEGATAG_Y_STD_DEV_INDEX = 1;
	public static final int MEGATAG_YAW_STD_DEV_INDEX = 5;

	// Standard deviation array indices for Megatag2
	public static final int MEGATAG2_X_STD_DEV_INDEX = 6;
	public static final int MEGATAG2_Y_STD_DEV_INDEX = 7;
	public static final int MEGATAG2_YAW_STD_DEV_INDEX = 11;

	// Validation constants
	public static final int EXPECTED_STD_DEV_ARRAY_LENGTH = 12;
	public static final int PER_MEGATAG_STD_DEV_COUNT = 6;

	public CameraConfiguration[] cameraConfigurations;
	/**
	 * Filtered AprilTag data for gyro correction
	 * When using gyro correction, only this subset of the AprilTags on the field
	 * are used
	 */
	public Optional<AprilTagFieldLayout> filteredAprilTagDataForGyro;
	public AprilTagFieldLayout aprilTagFieldLayout;

	/**
	 * Throttle value used while the robot is ENABLED.
	 * 0 means process every available frame (max update rate / lowest latency),
	 * but also highest CPU usage and heat.
	 */
	public final int ENABLED_CAMERA_THROTTLE = 0;

	/**
	 * Throttle value used while the robot is DISABLED.
	 * A positive value means "skip this many frames between processing",
	 * reducing CPU usage and heat at the cost of slower vision updates.
	 */
	public final int DISABLED_CAMERA_THROTTLE = 150;


	/**
	 * A very large variance used to down-weight or effectively reject unreliable
	 * measurements.
	 * <p>
	 * Unit: variance in the same units as the underlying state (e.g., m^2 or
	 * rad^2).
	 * Default: 1e6.
	 * </p>
	 */
	public final double largeVariance = 1e6;

	public double minMegatag1Quality = 0.8;

	public double minMegatag2Quality = 0.7;

	/**
	 * Maximum allowable multi-tag solution ambiguity (unitless), above which
	 * the estimate is rejected. Lower is stricter. Typical: ~0.10–0.25.
	 */
	public double defaultAmbiguityThreshold = 0.19;

	/**
	 * Maximum allowable absolute yaw delta (degrees) between vision-implied yaw and
	 * the current gyro yaw before rejecting the measurement (for single-tag
	 * low-area shots).
	 */
	public double defaultYawDiffThreshold = 20.0;

	/**
	 * Minimum tag area (in normalized image units, as produced by your pipeline)
	 * required
	 * to engage the yaw cross-check logic. Below this area, yaw tends to be noisy.
	 */
	public double tagAreaThresholdForYawCheck = 2.0;

	/**
	 * Minimum tag area (image units) required for single-tag Megatag usage.
	 * Single-tag estimates below this are often ill-conditioned.
	 */
	public double tagMinAreaForSingleTagMegatag = 1.0;

	/**
	 * Maximum allowable |Z| difference (meters) between vision and expected robot
	 * height
	 * before rejecting the measurement. Helps filter solutions with incorrect floor
	 * alignment.
	 */
	public double defaultZThreshold = 0.2;

	/**
	 * Maximum allowable gyro yaw error (degrees) for accepting a vision
	 * measurement.
	 */
	public double maxGyroYawErrorDegs = 60;

	/**
	 * Look-back window (seconds) for computing recent yaw velocity.
	 * If the absolute yaw rate exceeded {@link #highYawVelocityRadS} within this
	 * window,
	 * time-sync may be unreliable and the measurement can be rejected.
	 */
	public final double highYawLookbackS = 0.3;

	/**
	 * Yaw-rate threshold (radians/second) considered "high". Above this,
	 * measurement rejection
	 * is advisable because rolling-shutter/pipeline latencies can bias the pose.
	 */
	public final double highYawVelocityRadS = 1.0;

	public final double highSpeedThresholdMetersPerSecond = 2.0;

	/**
	 * Maximum allowable translation jump (meters) between the odometry pose and
	 * vision
	 * measurement. Larger jumps will reset odometry instead of fusing.
	 */
	public double maxTranslationJumpMeters = 2.0;
	/**
	 * Maximum allowable yaw jump (degrees) between the odometry pose and vision
	 * measurement. Larger jumps will reset odometry instead of fusing.
	 */
	public double maxYawJumpDegrees = 90.0;

	public double maxAllowedYawAngularVelocityMegatag2 = 1.0;
	public double mg2ConvergenceDegs = 10.0;

	// --- Simulation settings ---

	// I mabe shold make these per-camera later
	/**
	 * Average pixel error to simulate in vision measurements (pixels).
	 */
	public double simAvgErrorPx = 0.85;
	/**
	 * Standard deviation of pixel error to simulate in vision measurements
	 * (pixels).
	 */
	public double simErrorStdDevPx = 0.5;

	public double simAvgLatencyMs = 30;
	public double simBaseStdDevXY = 0.2;
	public double simBaseStdDevYawDeg = 5.0;

}
