package frc.DELib26.subsystems.swerve;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.units.Units;
import frc.DELib26.subsystems.swerve.swerveUtil.SwerveConstants;
import org.ironmaple.simulation.SimulatedArena;
import org.ironmaple.utils.mathutils.GeometryConvertor;

// If you notice issues with the swerve after pose resets, consider adding a small delay (e.g., Timer.delay(0.05)). However, be cautious as this might interfere with Photon simulation.
public class SwerveIOSim extends SwerveIOHardware {

	private static final double GRAVITY = 9.80665;
	private double lastUpdateTime = 0.0;
	private ChassisSpeeds lastSpeeds = new ChassisSpeeds(0.0, 0.0, 0.0);
	// Gravity in robot frame when flat on carpet & Pigeon aligned:
	double gx = 0.0;
	double gy = 0.0;

	public MapleSimSwerveDrivetrain mapleSimSwerveDrivetrain;

	public SwerveIOSim(SwerveConstants constants) {
		super(constants);
		this.mapleSimSwerveDrivetrain = new MapleSimSwerveDrivetrain(
				constants.ROBOT_MASS,
				constants.BUMPER_LENGTH,
				constants.BUMPER_WIDTH,
				constants.MODULE_HARDWARE_CONSTANTS.driveMotor,
				constants.MODULE_HARDWARE_CONSTANTS.steerMotor,
				constants.WHEEL_COEFFICIENT_OF_FRICTION.in(Units.Value),
				super.getModuleLocations(),
				super.getPigeon2(),
				super.getModules(),
				constants.SWERVE_MODULE_CONSTANTS
			);

		SimulatedArena.getInstance().addCustomSimulation(this.mapleSimSwerveDrivetrain);
	}

	@Override
	public void registerTelemetryFunction(SwerveIOInputs inputs) {
		super.registerTelemetry(state -> {
			state.Pose = mapleSimSwerveDrivetrain.mapleSimDrive.getSimulatedDriveTrainPose();
			inputs.logState(state);
		});
	}

	@Override
	public void updatePigeon2Inputs(Pigeon2IOInputs inputs) {
		//this update the timestamp and angular values
		super.updatePigeon2Inputs(inputs);

		ChassisSpeeds speeds = super.getStateCopy().Speeds;
		double dt = inputs.timestamp - this.lastUpdateTime;

		double ax = (speeds.vxMetersPerSecond - this.lastSpeeds.vxMetersPerSecond) / dt;
		double ay = (speeds.vyMetersPerSecond - this.lastSpeeds.vyMetersPerSecond) / dt;
		inputs.accelerationXG = (ax + gx) / GRAVITY;
		inputs.accelerationYG = (ay + gy) / GRAVITY;

		this.lastSpeeds = speeds;
		this.lastUpdateTime = inputs.timestamp;
	}

	@Override
	public void resetToParamaterizedRotation(Rotation2d rotation2d) {
		this.mapleSimSwerveDrivetrain.mapleSimDrive
			.getTransform()
			.setRotation(GeometryConvertor.toDyn4jRotation(rotation2d));
	}

	@Override
	public void resetRobotPose(Pose2d pose) {
		this.mapleSimSwerveDrivetrain.mapleSimDrive.setSimulationWorldPose(pose);

	}

	@Override
	public void resetRobotTranslation(Translation2d translation2d) {
		this.mapleSimSwerveDrivetrain.mapleSimDrive
			.getTransform()
			.setTranslation(GeometryConvertor.toDyn4jVector2(translation2d));

	}

	public MapleSimSwerveDrivetrain getMapleSimDrive() {
		return this.mapleSimSwerveDrivetrain;
	}
}