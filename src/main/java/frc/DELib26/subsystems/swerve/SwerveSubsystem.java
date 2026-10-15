package frc.DELib26.subsystems.swerve;

import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.swerve.SwerveDrivetrain;
import com.ctre.phoenix6.swerve.SwerveModule;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.ctre.phoenix6.swerve.utility.PhoenixPIDController;
import com.pathplanner.lib.util.DriveFeedforwards;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.DELib26.controller.GenericController;
import frc.DELib26.subsystems.swerve.SwerveIO.ModuleIOInputs;
import frc.DELib26.subsystems.swerve.SwerveIO.Pigeon2IOInputs;
import frc.DELib26.subsystems.swerve.SwerveIO.SwerveIOInputs;
import frc.DELib26.subsystems.swerve.swerveUtil.FieldOfMeritHelper;
import frc.DELib26.subsystems.swerve.swerveUtil.SwerveConstants;
import frc.DELib26.subsystems.swerve.swerveUtil.SwerveSubsystemConstants;
import frc.DELib26.subsystems.vision.visionUtil.VisionFieldPoseEstimate;
import frc.DELib26.util.FieldUtil;
import frc.DELib26.util.RobotState;
import org.littletonrobotics.junction.Logger;

import java.util.Arrays;
import java.util.function.Consumer;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.Value;

public class SwerveSubsystem extends SubsystemBase {
	public static final String KEY = "Subsystems/Swerve";
	// We have to put the PIDController here because we don't want to mess with
	// their lifecycle
	public final PIDController translationController;

	private final SwerveRequest.FieldCentricFacingAngle driveAtAngle = new SwerveRequest.FieldCentricFacingAngle()
			.withDriveRequestType(SwerveModule.DriveRequestType.Velocity);

	private final SwerveIOInputsAutoLogged swerveInputs = new SwerveIOInputsAutoLogged();

	private final ModuleIOInputsAutoLogged frontLeftInputs = new ModuleIOInputsAutoLogged();
	private final ModuleIOInputsAutoLogged frontRightInputs = new ModuleIOInputsAutoLogged();
	private final ModuleIOInputsAutoLogged backLeftInputs = new ModuleIOInputsAutoLogged();
	private final ModuleIOInputsAutoLogged backRightInputs = new ModuleIOInputsAutoLogged();

	private final Pigeon2IOInputsAutoLogged pigeon2Inputs = new Pigeon2IOInputsAutoLogged();

	private SwerveIO io;

	private final RobotState robotState;

	private final GenericController controller;
	private final FieldOfMeritHelper fomHelper;

	private Pose2d desiredPoseForDriveToPoint = new Pose2d();
	private Rotation2d desiredRotationForHeadingLock = new Rotation2d();

	private final double maxVelocity;
	private final double maxAngularVelocity;
	private final double SKEW_COMPENSATION_SCALAR;
	/*
	 * Coefficient to scale the maximum velocity during teleop driving
	 */
	private double teleopVelocityCoefficient = 1.0;
	/*
	 * Coefficient to scale the maximum angular velocity during teleop driving
	 */
	private double rotationVelocityCoefficient = 1.0;
	/**
	 * Maximum velocity to use when driving to a point
	 */
	private double maxVelocityForDriveToPoint;
	/*
	 * Maximum velocity to use when driving to a point
	 */
	private double maxAngularVelocityForDriveToPoint = Double.NaN;

	public SwerveSubsystem(
		SwerveIO io,
		GenericController controller,
		SwerveConstants swerveConstants,
		RobotState robotState
	) {
		this.io = io;
		this.controller = controller;
		this.maxVelocity = swerveConstants.MAX_LINEAR_SPEED.in(MetersPerSecond);
		this.maxAngularVelocity = swerveConstants.getMaxAngularSpeed().in(RadiansPerSecond);
		this.maxVelocityForDriveToPoint = swerveConstants.INITIAL_MAX_VELOCITY_FOR_DRIVE_TO_POINT.in(MetersPerSecond);
		this.maxAngularVelocityForDriveToPoint = swerveConstants.getInitialMaxAngularVelocityForDriveToPoint()
				.in(RadiansPerSecond);
		this.SKEW_COMPENSATION_SCALAR = swerveConstants.SKEW_COMPENSATION_SCALAR.in(Value);

		io.registerTelemetryFunction(swerveInputs);

		this.translationController = swerveConstants.TRANSLATION_CONTROLLER.toPIDController();

		this.driveAtAngle.HeadingController = swerveConstants.ROTATION_CONTROLLER.toPhoenixPIDController();
		this.driveAtAngle.HeadingController.enableContinuousInput(-Math.PI, Math.PI);

		this.fomHelper = new FieldOfMeritHelper(
			((SwerveDrivetrain<?, ?, ?>) this.io).getKinematics(),
			this.swerveInputs,
			this.pigeon2Inputs
		);

		this.robotState = robotState;
	}

	@Override
	public void periodic() {
		this.io.updateModuleInputs(
			this.frontLeftInputs,
			this.frontRightInputs,
			this.backLeftInputs,
			this.backRightInputs
		);
		this.io.updatePigeon2Inputs(this.pigeon2Inputs);
		this.io.setStandardDeviationsForState(
			this.fomHelper.getOdometryTrustMode(this.getRobotSpeeds()).getStateStdDevs()
		);
		this.robotState.updateFromSwerveInputs(this.swerveInputs, this.pigeon2Inputs);
	}

	public final void logTelemetry() {
		Logger.processInputs(KEY + "", this.swerveInputs);

		var command = this.getCurrentCommand();
		String commandName = command != null ? command.getName() : "None";
		Logger.recordOutput(KEY + "/Command", commandName);
	}

	public final void logExtendedTelemetry() {
		this.logTelemetry();
		Logger.processInputs(KEY + "/Module Data/Front Left", this.frontLeftInputs);
		Logger.processInputs(KEY + "/Module Data/Front Right", this.frontRightInputs);
		Logger.processInputs(KEY + "/Module Data/Back Left", this.backLeftInputs);
		Logger.processInputs(KEY + "/Module Data/Back Right", this.backRightInputs);

		Logger.processInputs(KEY + "/Pigeon2", this.pigeon2Inputs);

		this.fomHelper.log(KEY);

	}

	public boolean isManualDriveEnabled() {
		boolean areAxesActive = this.controller.areStickAxesActive(SwerveSubsystemConstants.CONTROLLER_DEADBAND);
		return !DriverStation.isAutonomous() && areAxesActive;
	}

	public void resetControllers() {
		this.translationController.reset();
		this.driveAtAngle.HeadingController.reset();
	}

	public void teleopDrive() {
		this.setSwerveState(new SwerveRequest.ApplyFieldSpeeds()
				.withSpeeds(this.calculateSpeedsBasedOnJoystickInputs())
				.withDriveRequestType(SwerveModule.DriveRequestType.Velocity));
	}

	// X_LOCK
	public void xLock() {
		this.setSwerveState(new SwerveRequest.SwerveDriveBrake());
	}

	// DRIVE_TO_POINT
	public void setMaxVelocityForDriveToPoint(double maxVelocityForDriveToPoint) {
		this.maxVelocityForDriveToPoint = maxVelocityForDriveToPoint;
	}

	public void setDesiredPoseForDriveToPoint(double maxAngularVelocityForDriveToPoint) {
		this.maxAngularVelocityForDriveToPoint = maxAngularVelocityForDriveToPoint;
	}

	public void setDesiredPoseForDriveToPoint(Pose2d pose) {
		this.desiredPoseForDriveToPoint = pose;

		Translation2d translationToDesiredPoint = this.desiredPoseForDriveToPoint
				.getTranslation()
				.minus(this.getPose().getTranslation());

		var linearDistance = translationToDesiredPoint.getNorm();
		var directionOfTravel = translationToDesiredPoint.getAngle();
		var velocityOutput = this.calculateVelocityOutputForDriveToPoint(
				this.translationController,
				linearDistance);

		var xComponent = velocityOutput * directionOfTravel.getCos();
		var yComponent = velocityOutput * directionOfTravel.getSin();

		String prefix = KEY + "/DriveToPoint";
		Logger.recordOutput(prefix + "/desiredPose", this.desiredPoseForDriveToPoint);
		Logger.recordOutput(prefix + "/xVelocitySetpoint", xComponent);
		Logger.recordOutput(prefix + "/yVelocitySetpoint", yComponent);
		Logger.recordOutput(prefix + "/velocityOutput", velocityOutput);
		Logger.recordOutput(prefix + "/linearDistance", linearDistance);
		Logger.recordOutput(prefix + "/directionOfTravel", directionOfTravel);

		this.driveAtAngle
				.withVelocityX(xComponent)
				.withVelocityY(yComponent)
				.withTargetDirection(this.desiredPoseForDriveToPoint.getRotation());

		if (Double.isFinite(this.maxAngularVelocityForDriveToPoint)) {
			this.driveAtAngle.withMaxAbsRotationalRate(this.maxAngularVelocityForDriveToPoint);
		}

		this.setSwerveState(this.driveAtAngle);
	}

	private double calculateVelocityOutputForDriveToPoint(PIDController controller, double linearDistance) {
		if (this.isAtDriveToPointSetpoint()) {
			return 0.0;
		}

		double friction = 0.0;
		if (linearDistance >= SwerveSubsystemConstants.DRIVE_TO_POINT_MIN_TRANSLATION_ERROR_FOR_FRICTION_METERS) {
			friction = SwerveSubsystemConstants.DRIVE_TO_POINT_STATIC_FRICTION_CONSTANT * this.maxVelocity;
		}

		double pid = controller.calculate(linearDistance, 0.0); // negative
		double mag = Math.min(Math.abs(pid) + friction, this.maxVelocityForDriveToPoint);

		if (mag < SwerveSubsystemConstants.DRIVE_TO_POINT_MIN_VELOCITY_FOR_ZERO_VALUE
				&& linearDistance < 2.0 * SwerveSubsystemConstants.DRIVE_TO_POINT_TRANSLATION_ERROR_TOLERANCE_METERS) {
			mag = 0.0;
		}
		return mag;
	}

	// Stop
	public void stop() {
		// ApplyRobotSpeeds resists movement by locking speeds to zero.
		// This is better than Idle, which allows the motor's neutral mode to take over
		// and can be pushed.
		this.setSwerveState(new SwerveRequest.ApplyRobotSpeeds());
	}

	// HeadingLock
	public void setDesiredHeadingForHeadingLock(Rotation2d heading) {
		this.setDesiredHeadingForHeadingLock(heading, 0.0);
	}

	public void setDesiredHeadingForHeadingLock(Rotation2d heading, double targetRateFeedforward) {
		this.desiredRotationForHeadingLock = heading;
		ChassisSpeeds speeds = this.calculateSpeedsBasedOnJoystickInputs();
		double rotationErrorToleranceDegrees = Math.toDegrees(this.driveAtAngle.HeadingController.getPositionTolerance());
		double angleDifferenceDegrees = FieldUtil.getAngleDifferenceDegrees(
			this.getPose().getRotation(),
			this.desiredRotationForHeadingLock
		);

		String prefix = KEY + "/HeadingLock";
		Logger.recordOutput(prefix + "/currentRotationDegrees", this.getPose().getRotation().getDegrees());
		Logger.recordOutput(prefix + "/desiredRotationDegrees", this.desiredRotationForHeadingLock.getDegrees());
		Logger.recordOutput(prefix + "/angleDifferenceDegrees", angleDifferenceDegrees);
		Logger.recordOutput(prefix + "/rotationErrorToleranceDegrees", rotationErrorToleranceDegrees);

		this.setSwerveState(this.driveAtAngle
				.withVelocityX(speeds.vxMetersPerSecond)
				.withVelocityY(speeds.vyMetersPerSecond)
				.withMaxAbsRotationalRate(0)
				.withTargetRateFeedforward(targetRateFeedforward)
				.withTargetDirection(heading)
			);
	}

	public void faceHeading(Rotation2d heading) {
		this.desiredRotationForHeadingLock = heading;

		this.setSwerveState(this.driveAtAngle
						.withVelocityX(0.0)
						.withVelocityY(0.0)
						.withMaxAbsRotationalRate(0)
						.withTargetRateFeedforward(0)
						.withTargetDirection(heading)
				);
	}

	public boolean isAtHeadingLockRotationSetpoint() {
		double rotationErrorToleranceDegrees = Math.toDegrees(this.driveAtAngle.HeadingController.getPositionTolerance());
		double angleDifferenceDegrees = FieldUtil.getAngleDifferenceDegrees(
			this.getPose().getRotation(),
			this.desiredRotationForHeadingLock
		);
		String prefix = KEY + "/HeadingLock";
		Logger.recordOutput(prefix + "/currentRotationDegrees", this.getPose().getRotation().getDegrees());
		Logger.recordOutput(prefix + "/desiredRotationDegrees", this.desiredRotationForHeadingLock.getDegrees());
		Logger.recordOutput(prefix + "/angleDifferenceDegrees", angleDifferenceDegrees);
		Logger.recordOutput(prefix + "/rotationErrorToleranceDegrees", rotationErrorToleranceDegrees);

		return angleDifferenceDegrees <= Math.abs(rotationErrorToleranceDegrees);
	}

	// POINT_WHEELS
	public void setDesiredAngleForPointWheelsState(double angleDegrees) {
		this.setSwerveState(new SwerveRequest.PointWheelsAt()
				.withModuleDirection(Rotation2d.fromDegrees(angleDegrees))
			);
	}

	// TELEOP_DRIVE
	private ChassisSpeeds calculateSpeedsBasedOnJoystickInputs() {
		if (DriverStation.getAlliance().isEmpty()) {
			return new ChassisSpeeds(0, 0, 0);
		}

		double xMagnitude = this.calculateMagnitude(this.controller.getLeftY());
		double yMagnitude = this.calculateMagnitude(this.controller.getLeftX());
		double angularMagnitude = this.calculateMagnitude(this.controller.getRightX());

		double dirMultiplier = FieldUtil.getAllianceDirectionMultiplier();

		double xVelocity = dirMultiplier * xMagnitude * this.maxVelocity * this.teleopVelocityCoefficient;
		double yVelocity = dirMultiplier * yMagnitude * this.maxVelocity * this.teleopVelocityCoefficient;
		double angularVelocity = angularMagnitude * this.maxAngularVelocity * this.rotationVelocityCoefficient;

		Rotation2d skewCompensationFactor = Rotation2d.fromRadians(
				this.getFieldSpeeds().omegaRadiansPerSecond * this.SKEW_COMPENSATION_SCALAR);

		Rotation2d robotAngle = this.swerveInputs.Pose.getRotation();
		var robotRel = ChassisSpeeds.fromFieldRelativeSpeeds(
				new ChassisSpeeds(xVelocity, yVelocity, angularVelocity),
				robotAngle);
		Logger.recordOutput(KEY + "/TeleopDrive/Comanded speeds", robotRel);
		return ChassisSpeeds.fromRobotRelativeSpeeds(robotRel, robotAngle.plus(skewCompensationFactor));
	}

	private double calculateMagnitude(double input) {
		double adjustedInput = -MathUtil.applyDeadband(input, SwerveSubsystemConstants.CONTROLLER_DEADBAND);
		adjustedInput = Math.copySign(Math.pow(adjustedInput, 2), adjustedInput);
		return adjustedInput;
	}

	// PATH_PLANNER_AUTO
	public void pathPlannerDriveAuto(ChassisSpeeds speeds, DriveFeedforwards ff) {
		Logger.recordOutput("PathPlanner/DriveAutoSpeeds", speeds);

		this.setSwerveState(new SwerveRequest.ApplyRobotSpeeds()
				.withDriveRequestType(SwerveModule.DriveRequestType.Velocity)
				.withDesaturateWheelSpeeds(true)
				.withSpeeds(speeds)
				.withWheelForceFeedforwardsX(ff.robotRelativeForcesXNewtons())
				.withWheelForceFeedforwardsY(ff.robotRelativeForcesYNewtons())
		);
	}

	public void resetPose(Pose2d pose2d) {
		this.io.resetRobotPose(pose2d);
		this.swerveInputs.Pose = pose2d;
		this.robotState.setLastYawAnchor(pose2d.getRotation(), Utils.getCurrentTimeSeconds());
	}

	public void resetTranslation(Pose2d pose) {
		this.io.resetRobotTranslation(pose.getTranslation());
		this.swerveInputs.Pose = new Pose2d(pose.getTranslation(), this.swerveInputs.Pose.getRotation());
	}

	public void resetRotation(Rotation2d rotation2d) {
		this.io.resetToParamaterizedRotation(rotation2d);
		this.swerveInputs.Pose = new Pose2d(this.swerveInputs.Pose.getTranslation(), rotation2d);
		this.swerveInputs.RawHeading = rotation2d;
		this.robotState.setLastYawAnchor(rotation2d, Utils.getCurrentTimeSeconds());
	}

	public void resetRotationToAllianceForward() {
		this.resetRotation(FieldUtil.getAllianceForwardHeading());
	}

	public boolean isAtDriveToPointSetpoint() {
		var distance = this.getDistanceFromDriveToPointSetpoint();
		return MathUtil.isNear(
				0.0, distance, SwerveSubsystemConstants.DRIVE_TO_POINT_TRANSLATION_ERROR_TOLERANCE_METERS);
	}

	public double getDistanceFromDriveToPointSetpoint() {
		return this.desiredPoseForDriveToPoint
				.getTranslation()
				.getDistance(this.getPose().getTranslation());
	}

	public boolean isAtDriveToPointDesiredRotation() {
		return FieldUtil.getAngleDifferenceDegrees(
			this.getPose(),
			this.desiredPoseForDriveToPoint
			) <= SwerveSubsystemConstants.DRIVE_TO_POINT_HEADING_ERROR_TOLERANCE_DEGREES;
	}

	public void setTeleopVelocityCoefficient(double teleopVelocityCoefficient) {
		this.teleopVelocityCoefficient = teleopVelocityCoefficient;
	}

	public void setRotationVelocityCoefficient(double rotationVelocityCoefficient) {
		this.rotationVelocityCoefficient = rotationVelocityCoefficient;
	}

	public void addVisionMeasurement(VisionFieldPoseEstimate visionFieldPoseEstimate) {
		this.io.addVisionMeasurement(visionFieldPoseEstimate);
	}

	public PhoenixPIDController getHeadingController() {
		return this.driveAtAngle.HeadingController;
	}

	// Input retrieval
	public Pose2d getPose() {
		return this.swerveInputs.Pose;
	}

	public ChassisSpeeds getRobotSpeeds() {
		return this.swerveInputs.RobotSpeeds;
	}

	public ChassisSpeeds getFieldSpeeds() {
		return ChassisSpeeds.fromRobotRelativeSpeeds(
				this.getRobotSpeeds(),
				this.getPose().getRotation());
	}

	// API
	public void setSwerveState(SwerveRequest request) {
		this.io.setSwerveState(request);
	}

	public void forEachMotor(Consumer<TalonFX> motorConsumer) {
		this.forEachDriveMotor(motorConsumer);
		this.forEachSteerMotor(motorConsumer);
	}

	public void forEachDriveMotor(Consumer<TalonFX> motorConsumer) {
		this.io.forEachDriveMotor(motorConsumer);
	}

	public void forEachSteerMotor(Consumer<TalonFX> motorConsumer) {
		this.io.forEachSteerMotor(motorConsumer);
	}

	public SwerveIOInputs getSwerveInputs() {
		return this.swerveInputs;
	}

	public ModuleIOInputs[] getModuleInputsArray() {
		return new ModuleIOInputs[] {
				this.frontLeftInputs,
				this.frontRightInputs,
				this.backLeftInputs,
				this.backRightInputs
		};
	}

	public double[] getWheelRadiusCharacterizationPositionsMeters() {
		return Arrays.stream(this.getSwerveInputs().ModulePositions)
				.mapToDouble(pos -> pos.distanceMeters)
				.toArray();
	}

	public Pigeon2IOInputs getPigeon2Inputs() {
		return this.pigeon2Inputs;
	}
}
