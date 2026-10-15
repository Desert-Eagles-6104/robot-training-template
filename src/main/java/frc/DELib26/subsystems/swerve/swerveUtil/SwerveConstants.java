package frc.DELib26.subsystems.swerve.swerveUtil;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.swerve.SwerveModuleConstants;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularAcceleration;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Dimensionless;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearAcceleration;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.Mass;
import edu.wpi.first.units.measure.MomentOfInertia;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.DELib26.motors.PIDControllerContainer;
import frc.DELib26.subsystems.swerve.MapleSimSwerveDrivetrain;

/**
 * Central configuration for a swerve drivetrain.
 *
 * <p>This class holds:
 * <ul>
 *     <li>Identity &amp; CAN bus configuration</li>
 *     <li>Module device IDs and CANcoder offsets</li>
 *     <li>Robot mass and geometry (for kinematics &amp; simulation)</li>
 *     <li>Motion limits and drive-to-point constraints</li>
 *     <li>SysId characterization configs</li>
 *     <li>Drive/steer PID and choreo/auto controllers</li>
 *     <li>Module &amp; drivetrain constants for CTRE Phoenix 6 swerve</li>
 * </ul>
 *
 * <p>Most physical quantities use the WPILib units API
 * (e.g. {@link Distance}, {@link LinearVelocity}, {@link Angle}, {@link Current})
 * so the underlying units can be changed centrally without renaming fields.
 */
public abstract class SwerveConstants {

	// ---------------------------------------------------------------------
	// Identity & CAN bus
	// ---------------------------------------------------------------------
	//#region Identity & CAN bus
	/** MAC address of the robot, used by Phoenix for swerve configuration selection. */
	public String MAC_ADDRESS;

	/** CAN bus name used by the swerve drivetrain (e.g. "rio", "canivore"). */
	public String CANBUS_NAME;

	/** Hardware constants of the chosen COTS swerve module (gear ratios, wheel radius, etc.). */
	public COTSTalonFXSwerveConstants MODULE_HARDWARE_CONSTANTS;
	//#endregion

	// ---------------------------------------------------------------------
	// Ports & device IDs
	// ---------------------------------------------------------------------
	//#region Ports & device IDs

	/** Pigeon / gyro CAN ID. */
	public int GYRO_ID;

	// --- Front left module device IDs ---
	public int FRONT_LEFT_DRIVE_MOTOR_ID;
	public int FRONT_LEFT_STEER_MOTOR_ID;
	public int FRONT_LEFT_STEER_ENCODER_ID;

	// --- Front right module device IDs ---
	public int FRONT_RIGHT_DRIVE_MOTOR_ID;
	public int FRONT_RIGHT_STEER_MOTOR_ID;
	public int FRONT_RIGHT_STEER_ENCODER_ID;

	// --- Back left module device IDs ---
	public int BACK_LEFT_DRIVE_MOTOR_ID;
	public int BACK_LEFT_STEER_MOTOR_ID;
	public int BACK_LEFT_STEER_ENCODER_ID;

	// --- Back right module device IDs ---
	public int BACK_RIGHT_DRIVE_MOTOR_ID;
	public int BACK_RIGHT_STEER_MOTOR_ID;
	public int BACK_RIGHT_STEER_ENCODER_ID;
	//#endregion

	// ---------------------------------------------------------------------
	// Mass & geometry (field coordinates, simulation, MapleSim)
	// ---------------------------------------------------------------------
	//#region Mass & geometry (field coordinates, simulation, MapleSim)

	/** The mass of the robot, including bumpers and battery */
	public Mass ROBOT_MASS;

	/** Robot bumpers width (Y dimension, left-right). */
	public Distance BUMPER_WIDTH;

	/** Robot bumpers length (X dimension, front-back). */
	public Distance BUMPER_LENGTH;

	/** Moment of inertia about the vertical (yaw) axis in kg·m². */
	public MomentOfInertia ROBOT_MOMENT_OF_INERTIA;

	/**
	 * Simulated moment of inertia of the *drive side* of each swerve module
	 * (wheel + geartrain + motor rotor), in kg·m².
	 * Used for CTRE swerve / drivetrain simulation (withDriveInertia).
	 */
	public MomentOfInertia SIM_DRIVE_MOMENT_OF_INERTIA;

	/**
	 * Coefficient of friction of the wheels on the carpet.
	 *
	 * <p>How to find:
	 * <ol>
	 *     <li>Weigh the robot to find the normal force (N).</li>
	 *     <li>Push the robot with a spring scale until it starts to move, record that force (F).</li>
	 *     <li>Compute μ = F / N and use that value here.</li>
	 * </ol>
	 */
	public Dimensionless WHEEL_COEFFICIENT_OF_FRICTION;

	/**
	 * Wheelbase: distance between front and back wheel centers (X direction).
	 */
	public Distance WHEELBASE_LENGTH;

	/**
	 * Track width: distance between left and right wheel centers (Y direction).
	 */
	public Distance WHEEL_TRACK_WIDTH;

	/**
	 * Robot gyro mounting angle relative to the robot reference frame.
	 * Typically expressed in degrees in configs, but stored as an {@link Angle} here.
	 */
	public Angle GYRO_MOUNTING_ANGLE;

	/**
	 * Scalar to compensate for skew when driving straight
	 * (e.g. imperfect center of rotation, uneven weight distribution).
	 *
	 * <p>Should be near 1.0. Values &gt; 1.0 or &lt; 1.0 can bias the drive
	 * outputs to keep the robot straight under load.
	 */
	public Dimensionless SKEW_COMPENSATION_SCALAR;
	//#endregion

	// ---------------------------------------------------------------------
	// Motion limits / drive-to-point constraints
	// ---------------------------------------------------------------------
	//#region Motion limits / drive-to-point constraints

	/** Maximum linear speed of the robot. */
	public LinearVelocity MAX_LINEAR_SPEED;

	/**
	 * Initial maximum linear speed used by drive-to-point controllers.
	 * This can be lower than {@link #MAX_LINEAR_SPEED} for smoother approach.
	 */
	public LinearVelocity INITIAL_MAX_VELOCITY_FOR_DRIVE_TO_POINT;

	/**
	 * Initial maximum angular velocity used by drive-to-point controllers.
	 * If left {@code null}, upstream code should provide a sane default.
	 */
	public AngularVelocity INITIAL_MAX_ANGULAR_VELOCITY_FOR_DRIVE_TO_POINT;

	public LinearAcceleration MAX_LINEAR_ACCELERATION;

	public AngularAcceleration MAX_ANGULAR_ACCELERATION;

	/**
	 * Cached maximum angular speed computed from {@link #MAX_LINEAR_SPEED}
	 * and the robot radius from center to module.
	 */
	private AngularVelocity MAX_ANGULAR_SPEED = null;

	/**
	 * Returns the maximum angular speed (rad/s) based on
	 * the current {@link #MAX_LINEAR_SPEED}, {@link #WHEELBASE_LENGTH},
	 * and {@link #WHEEL_TRACK_WIDTH}.
	 */
	public AngularVelocity getMaxAngularSpeed() {
		if (MAX_ANGULAR_SPEED == null) {
			double hx = WHEELBASE_LENGTH.in(Units.Meters) / 2.0;
			double hy = WHEEL_TRACK_WIDTH.in(Units.Meters) / 2.0;

			// Effective radius from robot center to a module
			double radiusMeters = Math.hypot(hx, hy);

			// ω = v / r
			double maxSpeedMps = MAX_LINEAR_SPEED.in(Units.MetersPerSecond);
			double maxAngularRadPerSec = maxSpeedMps / radiusMeters;

			MAX_ANGULAR_SPEED = Units.RadiansPerSecond.of(maxAngularRadPerSec);
		}
		return MAX_ANGULAR_SPEED;
	}

	/**
	 * Returns the initial maximum linear velocity for drive-to-point.
	 * Defaults to {@link #getMaxAngularSpeed()} if not set.
	 */
	public AngularVelocity getInitialMaxAngularVelocityForDriveToPoint() {
		if (INITIAL_MAX_ANGULAR_VELOCITY_FOR_DRIVE_TO_POINT != null) {
			return INITIAL_MAX_ANGULAR_VELOCITY_FOR_DRIVE_TO_POINT;
		} else {
			// Default to max angular speed if not set
			return getMaxAngularSpeed();
		}
	}

	//#endregion

	// ---------------------------------------------------------------------
	// CANcoder / module steering offsets
	// ---------------------------------------------------------------------
	//#region CANcoder / module steering offsets

	/**
	 * Absolute CANcoder offset for the front-left module steer axis.
	 * This is the mechanical zero angle when the wheels are pointing forward.
	 */
	public Angle FRONT_LEFT_STEER_OFFSET;

	/** Absolute CANcoder offset for the front-right module steer axis. */
	public Angle FRONT_RIGHT_STEER_OFFSET;

	/** Absolute CANcoder offset for the back-left module steer axis. */
	public Angle BACK_LEFT_STEER_OFFSET;

	/** Absolute CANcoder offset for the back-right module steer axis. */
	public Angle BACK_RIGHT_STEER_OFFSET;
	//#endregion

	// ---------------------------------------------------------------------
	// SysId characterization configs
	// ---------------------------------------------------------------------
	//#region SysId characterization configs

	/** SysId configuration for drivetrain translation (linear) characterization. */
	public SysIdRoutine.Config TRANSLATION_SYS_ID_CONFIG;

	/** SysId configuration for drivetrain rotation (yaw) characterization. */
	public SysIdRoutine.Config ROTATION_SYS_ID_CONFIG;

	public SysIdRoutine.Config STEER_SYS_ID_CONFIG;
	//#endregion

	// ---------------------------------------------------------------------
	// PID & controller containers
	// ---------------------------------------------------------------------
	//#region PID & controller containers

	/** Slot0 (drive) configuration for CTRE TalonFX drive motors. */
	public Slot0Configs DRIVE_SLOT0_CONFIGS;

	/** Slot0 (steer) configuration for CTRE TalonFX steer motors. */
	public Slot0Configs STEER_SLOT0_CONFIGS;

	/** PID controller for robot translation (X/Y). */
	public PIDControllerContainer TRANSLATION_CONTROLLER;

	/** PID controller for robot rotation (yaw). */
	public PIDControllerContainer ROTATION_CONTROLLER;

	public PIDControllerContainer PATH_PLANNER_TRANSLATION_CONTROLLER;

	public PIDControllerContainer PATH_PLANNER_ROTATION_CONTROLLER;

	//#endregion

	// ---------------------------------------------------------------------
	// Current limits
	// ---------------------------------------------------------------------
	//#region Current limits

	/** Drive motor stator current limit. */
	public Current DRIVE_STATOR_CURRENT_LIMIT;

	/** Steer motor stator current limit. */
	public Current STEER_STATOR_CURRENT_LIMIT;
	//#endregion

	// ---------------------------------------------------------------------
	// Swerve modules & drivetrain definitions
	// ---------------------------------------------------------------------
	//#region Swerve modules & drivetrain definitions

	/**
	 * Per-module constants used by the Phoenix 6 swerve API.
	 * <p>Order: FL, FR, BL, BR.
	 */
	public SwerveModuleConstants<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration>[]
			SWERVE_MODULE_CONSTANTS;

	/** Drivetrain-level CTRE swerve constants (Pigeon, CAN bus, etc.). */
	public SwerveDrivetrainConstants SWERVE_DRIVETRAIN_CONSTANTS;

	/**
	 * Constructs a base set of {@link SwerveModuleConstants} using:
	 * <ul>
	 *     <li>Default drive/steer configs from {@link #getDefaultDriveTalonFXConfig()} and
	 *     {@link #getDefaultSteerTalonFXConfig()}</li>
	 *     <li>Gear ratios and inversion from {@link #MODULE_HARDWARE_CONSTANTS}</li>
	 *     <li>Slot0 drive and steer gains from {@link #DRIVE_SLOT0_CONFIGS} and
	 *     {@link #STEER_SLOT0_CONFIGS}</li>
	 * </ul>
	 *
	 * <p>Additional per-module IDs, offsets, and locations are applied in
	 * {@link #getDefaultSwerveModuleConstantsArray()}.
	 */
	protected SwerveModuleConstants<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration>
			getDefaultSwerveModuleConstants() {

		return new SwerveModuleConstants<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration>()
				.withDriveMotorInitialConfigs(getDefaultDriveTalonFXConfig())
				.withSteerMotorInitialConfigs(getDefaultSteerTalonFXConfig())
				.withDriveMotorType(SwerveModuleConstants.DriveMotorArrangement.TalonFX_Integrated)
				.withSteerMotorType(SwerveModuleConstants.SteerMotorArrangement.TalonFX_Integrated)
				.withDriveMotorClosedLoopOutput(SwerveModuleConstants.ClosedLoopOutputType.Voltage)
				.withSteerMotorClosedLoopOutput(SwerveModuleConstants.ClosedLoopOutputType.Voltage)
				.withDriveMotorGains(DRIVE_SLOT0_CONFIGS)
				.withSteerMotorGains(STEER_SLOT0_CONFIGS)
				.withDriveMotorGearRatio(MODULE_HARDWARE_CONSTANTS.driveGearRatio)
				.withSteerMotorGearRatio(MODULE_HARDWARE_CONSTANTS.steerGearRatio)
				.withCouplingGearRatio(MODULE_HARDWARE_CONSTANTS.couplingGearRatio)
				.withDriveMotorInverted(MODULE_HARDWARE_CONSTANTS.driveMotorInvert)
				.withSteerMotorInverted(MODULE_HARDWARE_CONSTANTS.steerMotorInvert)
				.withEncoderInverted(MODULE_HARDWARE_CONSTANTS.cancoderInvert)
				.withEncoderInitialConfigs(new CANcoderConfiguration())
				// Fused CANcoder is Pro-only; we use remote CANcoder feedback instead.
				.withFeedbackSource(SwerveModuleConstants.SteerFeedbackType.RemoteCANcoder)
				// Phoenix expects the theoretical max speed at 12V (no friction / slip reduction).
				.withSpeedAt12Volts(MAX_LINEAR_SPEED)
				.withWheelRadius(MODULE_HARDWARE_CONSTANTS.getWheelRadius())
				.withSlipCurrent(120) // Reasonable default; tune as needed.
				// For sim: steer inertia and drive/steer friction are auto-assigned in
				// {@link MapleSimSwerveDrivetrain#regulateModuleConstantsForSimulation(SwerveModuleConstants[])}
				.withDriveInertia(SIM_DRIVE_MOMENT_OF_INERTIA);
	}

	/**
	 * Returns default TalonFX configuration for all drive motors.
	 */
	protected abstract TalonFXConfiguration getDefaultDriveTalonFXConfig();

	/**
	 * Returns default TalonFX configuration for all steer motors.
	 */
	protected abstract TalonFXConfiguration getDefaultSteerTalonFXConfig();

	/**
	 * Builds the array of module constants (FL, FR, BL, BR) with:
	 * <ul>
	 *     <li>Per-module motor / encoder IDs</li>
	 *     <li>Per-module absolute steer offsets</li>
	 *     <li>Per-module locations in robot coordinates</li>
	 *     <li>Simulation regulation via MapleSim</li>
	 * </ul>
	 */
	@SuppressWarnings("unchecked")
	protected SwerveModuleConstants<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration>[]
			getDefaultSwerveModuleConstantsArray() {

		double halfWheelbaseMeters = WHEELBASE_LENGTH.in(Units.Meters) / 2.0;
		double halfTrackWidthMeters = WHEEL_TRACK_WIDTH.in(Units.Meters) / 2.0;

		return MapleSimSwerveDrivetrain.regulateModuleConstantsForSimulation(new SwerveModuleConstants[] {
				getDefaultSwerveModuleConstants()
						.withDriveMotorId(FRONT_LEFT_DRIVE_MOTOR_ID)
						.withSteerMotorId(FRONT_LEFT_STEER_MOTOR_ID)
						.withEncoderId(FRONT_LEFT_STEER_ENCODER_ID)
						.withEncoderOffset(FRONT_LEFT_STEER_OFFSET)
						.withLocationX(halfWheelbaseMeters)
						.withLocationY(halfTrackWidthMeters),

				getDefaultSwerveModuleConstants()
						.withDriveMotorId(FRONT_RIGHT_DRIVE_MOTOR_ID)
						.withSteerMotorId(FRONT_RIGHT_STEER_MOTOR_ID)
						.withEncoderId(FRONT_RIGHT_STEER_ENCODER_ID)
						.withEncoderOffset(FRONT_RIGHT_STEER_OFFSET)
						.withLocationX(halfWheelbaseMeters)
						.withLocationY(-halfTrackWidthMeters),

				getDefaultSwerveModuleConstants()
						.withDriveMotorId(BACK_LEFT_DRIVE_MOTOR_ID)
						.withSteerMotorId(BACK_LEFT_STEER_MOTOR_ID)
						.withEncoderId(BACK_LEFT_STEER_ENCODER_ID)
						.withEncoderOffset(BACK_LEFT_STEER_OFFSET)
						.withLocationX(-halfWheelbaseMeters)
						.withLocationY(halfTrackWidthMeters),

				getDefaultSwerveModuleConstants()
						.withDriveMotorId(BACK_RIGHT_DRIVE_MOTOR_ID)
						.withSteerMotorId(BACK_RIGHT_STEER_MOTOR_ID)
						.withEncoderId(BACK_RIGHT_STEER_ENCODER_ID)
						.withEncoderOffset(BACK_RIGHT_STEER_OFFSET)
						.withLocationX(-halfWheelbaseMeters)
						.withLocationY(-halfTrackWidthMeters)
		});
	}
	//#endregion

	// ---------------------------------------------------------------------
	// Helper accessors
	// ---------------------------------------------------------------------
	//#region Helper accessors

	/**
	 * Returns an array of module locations as {@link Translation2d}:
	 * <pre>
	 * index 0: front-left ( +X, +Y )
	 * index 1: front-right( +X, -Y )
	 * index 2: back-left  ( -X, +Y )
	 * index 3: back-right ( -X, -Y )
	 * </pre>
	 */
	public Translation2d[] getModuleLocations() {
		double hx = WHEELBASE_LENGTH.in(Units.Meters) / 2.0;
		double hy = WHEEL_TRACK_WIDTH.in(Units.Meters) / 2.0;

		return new Translation2d[] {
				new Translation2d(+hx, +hy), // FL
				new Translation2d(+hx, -hy), // FR
				new Translation2d(-hx, +hy), // BL
				new Translation2d(-hx, -hy)  // BR
		};
	}

	public double computeWheelBaseDiagonal() {
		return Math.hypot(WHEELBASE_LENGTH.in(Units.Meters) / 2.0, WHEEL_TRACK_WIDTH.in(Units.Meters) / 2.0);
	}
}
