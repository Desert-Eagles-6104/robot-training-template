package frc.DELib26.subsystems.swerve.swerveUtil;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.DriverStation;
import frc.DELib26.subsystems.swerve.SwerveIO.Pigeon2IOInputs;
import frc.DELib26.subsystems.swerve.SwerveIO.SwerveIOInputs;
import org.littletonrobotics.junction.Logger;

public class FieldOfMeritHelper {
	public static final String BASE_KEY = "/FieldOfMerit";

	// References to things we need
	private final SwerveDriveKinematics kinematics;
	private final SwerveIOInputs swerveInputs;
	private final Pigeon2IOInputs pigeon2Inputs;

	private double lastSkidRatio;
	private boolean isSkidding;
	private boolean collision;
	private double robotSpeedMagnitude;
	private double accelAlongMotionG;

	private double filteredAccelXG = 0.0;
	private double filteredAccelYG = 0.0;

	public FieldOfMeritHelper(
		SwerveDriveKinematics kinematics,
		SwerveIOInputs swerveInputs,
		Pigeon2IOInputs pigeon2Inputs
	) {
		this.kinematics = kinematics;
		this.swerveInputs = swerveInputs;
		this.pigeon2Inputs = pigeon2Inputs;
	}

	// under development, not fully implemented yet.
	// Idea is to have a single place to determine how much we trust our odometry based on various factors like skidding and collisions, and then use that to adjust our state estimation uncertainty dynamically.
	public OdometryTrustMode getOdometryTrustMode(ChassisSpeeds robotSpeeds) {
		if (DriverStation.isDisabled()) {
			return OdometryTrustMode.DISABLED;
		}

		if (DriverStation.isAutonomous()) {
			//return OdometryTrustMode.AUTO;
		}
		this.isSkidding();

		this.detectCollision(robotSpeeds);

		return OdometryTrustMode.NORMAL;
	}

	public void log(String prefix) {
		Logger.recordOutput(prefix + BASE_KEY + "/isSkidding", isSkidding);
		Logger.recordOutput(prefix + BASE_KEY + "/accelMagG", Math.hypot(filteredAccelXG, filteredAccelYG));

		Logger.recordOutput(prefix + BASE_KEY + "/robotSpeedMagnitude", robotSpeedMagnitude);
		Logger.recordOutput(prefix + BASE_KEY + "/accelAlongMotionG", accelAlongMotionG);
		Logger.recordOutput(prefix + BASE_KEY + "/collision", collision);
	}

	public boolean isSkidding() {

		this.lastSkidRatio = this.getSkiddingRatio(
			this.swerveInputs.ModuleStates,
			this.swerveInputs.ModulePositions,
			this.kinematics
		);

		return this.lastSkidRatio > SwerveSubsystemConstants.FIELD_OF_MERIT_SKID_RATIO_THRESHOLD;
	}

	public boolean detectCollision( ChassisSpeeds robotSpeeds) {
		double accelX = this.pigeon2Inputs.accelerationXG;
		double accelY = this.pigeon2Inputs.accelerationYG;

		// Low-pass
		filteredAccelXG = calculateWeightedAverage(
			SwerveSubsystemConstants.FIELD_OF_MERIT_COLLISION_FILTER_ALPHA,
			filteredAccelXG,
			accelX
		);

		filteredAccelYG = calculateWeightedAverage(
			SwerveSubsystemConstants.FIELD_OF_MERIT_COLLISION_FILTER_ALPHA,
			filteredAccelYG,
			accelY
		);

		double vx = robotSpeeds.vxMetersPerSecond;
		double vy = robotSpeeds.vyMetersPerSecond;
		this.robotSpeedMagnitude = Math.hypot(vx, vy);

		// Component of accel along direction of motion (in g)
		this.accelAlongMotionG = (filteredAccelXG * vx + filteredAccelYG * vy) / robotSpeedMagnitude;

		this.collision = accelAlongMotionG > SwerveSubsystemConstants.FIELD_OF_MERIT_COLLISION_THRESHOLD_G;

		return collision;
	}

	private static double calculateWeightedAverage(
		double weightFactor,
		double baseValue,
		double newValue
	) {
		return (1.0 - weightFactor) * baseValue + weightFactor * newValue;
	}

	/**
	 * Computes the skidding ratio of a swerve drive system, indicating wheel slip.
	 *
	 * @param measuredStates The measured {@link SwerveModuleState} array.
	 * @param kinematics     The {@link SwerveDriveKinematics} for calculations.
	 * @return Skidding ratio (1.0 = no skid, >1.0 = skidding).
	 *
	 *         <p>
	 *         Steps:
	 *         <ol>
	 *         <li>Calculate chassis rotational velocity (omega).</li>
	 *         <li>Compute rotation-only and translational-only vectors.</li>
	 *         <li>Find max, min, and mean translational speeds.</li>
	 *         <li>Handle edge cases for negligible speeds.</li>
	 *         <li>Return max/min translational speed ratio.</li>
	 *         </ol>
	 */
	public double getSkiddingRatio(
		SwerveModuleState[] measuredStates,
		SwerveModulePosition[] modulePositions,
		SwerveDriveKinematics kinematics
	) {
		ChassisSpeeds chassis = kinematics.toChassisSpeeds(measuredStates);
		double omega = chassis.omegaRadiansPerSecond;

		// Compute rotation-only states
		SwerveModuleState[] rotationStates = kinematics.toSwerveModuleStates(
			new ChassisSpeeds(0.0, 0.0, omega)
		);

		// Compute translational-only vectors per module
		double[] transMags = new double[measuredStates.length];
		for (int i = 0; i < measuredStates.length; i++) {
			// Current *measured* module angle (steer sensor)
			Rotation2d currentAngle = modulePositions[i].angle;

			// Copy then align speed sign + angle to the real current module angle
			SwerveModuleState measuredAligned = new SwerveModuleState(
					measuredStates[i].speedMetersPerSecond,
					measuredStates[i].angle
				);

			measuredAligned.optimize(currentAngle);

			Translation2d measuredVec = convertSwerveStateToVelocityVector(measuredAligned);
			Translation2d rotVec = convertSwerveStateToVelocityVector(rotationStates[i]);

			Translation2d transVec = measuredVec.minus(rotVec);
			transMags[i] = transVec.getNorm();
		}

		// Get max/min translational speeds
		double maxTrans = 0.0;
		double minTrans = Double.POSITIVE_INFINITY;
		double sumTrans = 0.0;

		for (double v : transMags) {
			maxTrans = Math.max(maxTrans, v);
			minTrans = Math.min(minTrans, v);
			sumTrans += v;
		}

		double meanTrans = sumTrans / measuredStates.length;

		//Compute max rotational wheel speed for scale
		double maxRotSpeed = 0.0;
		for (var s : rotationStates) {
			maxRotSpeed = Math.max(maxRotSpeed, Math.abs(s.speedMetersPerSecond));
		}

		if (maxTrans < SwerveSubsystemConstants.FIELD_OF_MERIT_SKID_ABS_TRANS_EPS) {
			return 1.0;
		}

		// avoid division by zero if not rotating
		if (maxRotSpeed > 1e-3 && (maxTrans / maxRotSpeed) < SwerveSubsystemConstants.FIELD_OF_MERIT_SKID_REL_TRANS_EPS) {
			return 1.0;
		}

		// If mean translational speed is tiny, same idea
		if (meanTrans < SwerveSubsystemConstants.FIELD_OF_MERIT_SKID_ABS_TRANS_EPS) {
			return 1.0;
		}

		// Avoid divide-by-zero or insane ratios from extremely small min
		if (minTrans < SwerveSubsystemConstants.FIELD_OF_MERIT_SKID_ABS_TRANS_EPS) {
			minTrans = SwerveSubsystemConstants.FIELD_OF_MERIT_SKID_ABS_TRANS_EPS;
		}

		return maxTrans / minTrans;
	}

	public double getLastSkidRatio() {
		return this.lastSkidRatio;
	}

	private static Translation2d convertSwerveStateToVelocityVector(SwerveModuleState swerveModuleState) {
		return new Translation2d(swerveModuleState.speedMetersPerSecond, swerveModuleState.angle);
	}

}
