package frc.DELib26.subsystems.swerve;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.swerve.SwerveDrivetrain;
import com.ctre.phoenix6.swerve.SwerveRequest;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import frc.DELib26.subsystems.vision.visionUtil.VisionFieldPoseEstimate;
import org.littletonrobotics.junction.AutoLog;

import java.util.function.Consumer;

public interface SwerveIO {

	@AutoLog
	class SwerveIOInputs {
		public Pose2d Pose = new Pose2d();
		/**
		 * The desired chassis speeds for the drivetrain, robot-relative.
		 */
		public ChassisSpeeds RobotSpeeds = new ChassisSpeeds();
		/**
		 * The desired chassis speeds for the drivetrain, field-relative.
		 */
		// public ChassisSpeeds FieldSpeeds = new ChassisSpeeds();
		public SwerveModuleState[] ModuleStates;
		public SwerveModuleState[] ModuleTargets;
		public SwerveModulePosition[] ModulePositions;
		public Rotation2d RawHeading = new Rotation2d();
		//FPGA timestamp of the last odometry update
		public double Timestamp;
		public double OdometryPeriod;
		public int SuccessfulDaqs;
		public int FailedDaqs;

		void logState(SwerveDrivetrain.SwerveDriveState state) {
			this.Pose = state.Pose;
			this.RawHeading = state.RawHeading;
			this.ModuleStates = state.ModuleStates;
			this.ModuleTargets = state.ModuleTargets;
			this.ModulePositions = state.ModulePositions;
			this.RobotSpeeds = state.Speeds;
			this.SuccessfulDaqs = state.SuccessfulDaqs;
			this.FailedDaqs = state.FailedDaqs;
			this.OdometryPeriod = state.OdometryPeriod;
			this.Timestamp = state.Timestamp;
		}
	}

	@AutoLog
	class ModuleIOInputs {
		public double driveSupplyCurrentAmps = 0.0;
		public double driveStatorCurrentAmps = 0.0;
		public double driveAppliedVolts = 0.0;
		public double driveTemperature = 0.0;

		public double steerSupplyCurrentAmps = 0.0;
		public double steerStatorCurrentAmps = 0.0;
		public double steerAppliedVolts = 0.0;
		public double steerTemperature = 0.0;
	}
	// Units dps = degrees per second
	@AutoLog
	class Pigeon2IOInputs {
		public double timestamp = 0.0;

		public double accelerationXG = 0.0;
		public double accelerationYG = 0.0;

		public double angularRollVelocityDPS = 0.0;
		public double angularPitchVelocityDPS = 0.0;
		public double angularYawVelocityDPS = 0.0;
		public double rollDegs = 0.0;
		public double pitchDegs = 0.0;
		public double yawDegs = 0.0;
	}


	void updateModuleInputs(ModuleIOInputs... inputs);

	void updatePigeon2Inputs(Pigeon2IOInputs inputs);

	void registerTelemetryFunction(SwerveIOInputs inputs);

	void refreshData();

	void setSwerveState(SwerveRequest request);

	void resetRobotPose(Pose2d pose);

	void resetToParamaterizedRotation(Rotation2d rotation2d);

	void updateSimState();

	void resetRobotTranslation(Translation2d translation2d);

	void disableMotors();

	default void setStandardDeviationsForState(double xStd, double yStd, double rotStd){
		this.setStandardDeviationsForState(VecBuilder.fill(xStd, yStd, rotStd));
	}

	void setStandardDeviationsForState(Matrix<N3, N1> stateStdDevs);

	void addVisionMeasurement(VisionFieldPoseEstimate visionFieldPoseEstimate);

	void forEachDriveMotor(Consumer<TalonFX> consumer);

	void forEachSteerMotor(Consumer<TalonFX> consumer);
}
