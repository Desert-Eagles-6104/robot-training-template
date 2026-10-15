package frc.DELib26.subsystems.swerve;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.swerve.SwerveDrivetrain;
import com.ctre.phoenix6.swerve.SwerveRequest;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.RobotController;
import frc.DELib26.subsystems.swerve.swerveUtil.SwerveConstants;
import frc.DELib26.subsystems.vision.visionUtil.VisionFieldPoseEstimate;
import frc.DELib26.util.ProjectConstants;

import java.util.HashMap;
import java.util.Map;
import java.util.function.Consumer;

public class SwerveIOHardware extends SwerveDrivetrain<TalonFX, TalonFX, CANcoder> implements SwerveIO {

	private HashMap<String, BaseStatusSignal> frontLeftSignals = new HashMap<>();
	private HashMap<String, BaseStatusSignal> frontRightSignals = new HashMap<>();
	private HashMap<String, BaseStatusSignal> backLeftSignals = new HashMap<>();
	private HashMap<String, BaseStatusSignal> backRightSignals = new HashMap<>();

	private HashMap<String, BaseStatusSignal> pigeon2Signals = new HashMap<>();

	private Map<Integer, HashMap<String, BaseStatusSignal>> signalsMap = new HashMap<>();

	public SwerveIOHardware(SwerveConstants constants) {
		super(
			TalonFX::new,
			TalonFX::new,
			CANcoder::new,
			constants.SWERVE_DRIVETRAIN_CONSTANTS,
			50.0,
			constants.SWERVE_MODULE_CONSTANTS
		);

		signalsMap.put(0, frontLeftSignals);
		signalsMap.put(1, frontRightSignals);
		signalsMap.put(2, backLeftSignals);
		signalsMap.put(3, backRightSignals);

		for (int i = 0; i < 4; i++) {
			var driveMotor = this.getModule(i).getDriveMotor();
			var steerMotor = this.getModule(i).getSteerMotor();

			var moduleMap = signalsMap.get(i);

			moduleMap.put("driveSupplyCurrentAmps", driveMotor.getSupplyCurrent());
			moduleMap.put("driveStatorCurrentAmps", driveMotor.getStatorCurrent());
			moduleMap.put("driveAppliedVolts", driveMotor.getMotorVoltage());
			moduleMap.put("driveTemperature", driveMotor.getDeviceTemp());

			moduleMap.put("steerSupplyCurrentAmps", steerMotor.getSupplyCurrent());
			moduleMap.put("steerStatorCurrentAmps", steerMotor.getStatorCurrent());
			moduleMap.put("steerAppliedVolts", steerMotor.getMotorVoltage());
			moduleMap.put("steerTemperature", steerMotor.getDeviceTemp());
			BaseStatusSignal.setUpdateFrequencyForAll(10, moduleMap.values().toArray(new BaseStatusSignal[] {}));
		}

		this.pigeon2Signals.put("accelerationX", super.getPigeon2().getAccelerationX());
		this.pigeon2Signals.put("accelerationY", super.getPigeon2().getAccelerationY());

		this.pigeon2Signals.put("rollDegs", super.getPigeon2().getRoll());
		this.pigeon2Signals.put("pitchDegs", super.getPigeon2().getPitch());
		this.pigeon2Signals.put("yawDegs", super.getPigeon2().getYaw());

		this.pigeon2Signals.put("angularPitchVelocityDPS", super.getPigeon2().getAngularVelocityYWorld());
		this.pigeon2Signals.put("angularRollVelocityDPS", super.getPigeon2().getAngularVelocityXWorld());
		this.pigeon2Signals.put("angularYawVelocityDPS", super.getPigeon2().getAngularVelocityZWorld());
		BaseStatusSignal.setUpdateFrequencyForAll(
			50,
			this.pigeon2Signals.values().toArray(new BaseStatusSignal[] {})
		);
		super.getOdometryThread().setThreadPriority(99);
		super.setVisionMeasurementStdDevs(VecBuilder.fill(0.3, 0.3, 9999999));
		super.setStateStdDevs(VecBuilder.fill(0.1, 0.1, 0.1));
	}

	/**
	 *
	 * Registers a telemetry function to log the swerve drivetrain state to the provided inputs.
	 * Its frequency is controlled by the odometry update frequency.
	 */
	@Override
	public void registerTelemetryFunction(SwerveIOInputs inputs) {
		super.registerTelemetry(state -> {
			inputs.logState(state);
		});
	}

	@Override
public void updateModuleInputs(ModuleIOInputs... inputs) {
	this.refreshModulesData(); // add this

	for (int i = 0; i < 4; i++) {
		var moduleMap = signalsMap.get(i);

		inputs[i].driveSupplyCurrentAmps =
				moduleMap.get("driveSupplyCurrentAmps").getValueAsDouble();

		inputs[i].driveStatorCurrentAmps =
				moduleMap.get("driveStatorCurrentAmps").getValueAsDouble();

		inputs[i].driveAppliedVolts =
				moduleMap.get("driveAppliedVolts").getValueAsDouble();

		inputs[i].driveTemperature =
				moduleMap.get("driveTemperature").getValueAsDouble();

		inputs[i].steerSupplyCurrentAmps =
				moduleMap.get("steerSupplyCurrentAmps").getValueAsDouble();

		inputs[i].steerStatorCurrentAmps =
				moduleMap.get("steerStatorCurrentAmps").getValueAsDouble();

		inputs[i].steerAppliedVolts =
				moduleMap.get("steerAppliedVolts").getValueAsDouble();

		inputs[i].steerTemperature =
				moduleMap.get("steerTemperature").getValueAsDouble();
	}
}

	@Override
	public void updatePigeon2Inputs(Pigeon2IOInputs inputs) {
		this.refreshPigeon2Data();
		var pigeonMap = this.pigeon2Signals;

		inputs.timestamp = Utils.getCurrentTimeSeconds();
		inputs.accelerationXG = pigeonMap.get("accelerationX").getValueAsDouble();
		inputs.accelerationYG = pigeonMap.get("accelerationY").getValueAsDouble();

		inputs.angularRollVelocityDPS = pigeonMap.get("angularRollVelocityDPS").getValueAsDouble();
		inputs.angularPitchVelocityDPS = pigeonMap.get("angularPitchVelocityDPS").getValueAsDouble();
		inputs.angularYawVelocityDPS = pigeonMap.get("angularYawVelocityDPS").getValueAsDouble();

		inputs.rollDegs = pigeonMap.get("rollDegs").getValueAsDouble();
		inputs.pitchDegs = pigeonMap.get("pitchDegs").getValueAsDouble();
		inputs.yawDegs = pigeonMap.get("yawDegs").getValueAsDouble();
	}

	@Override
	public void setSwerveState(SwerveRequest request) {
		super.setControl(request);
	}

	@Override
	public void resetToParamaterizedRotation(Rotation2d rotation2d) {
		super.resetRotation(rotation2d);
	}

	@Override
	public void updateSimState() {
		super.updateSimState(ProjectConstants.DEFAULT_PERIOD_SECS, RobotController.getBatteryVoltage());
	}

	@Override
	public void resetRobotPose(Pose2d pose) {
		super.resetPose(pose);
	}

	@Override
	public void resetRobotTranslation(Translation2d translation2d) {
		super.resetTranslation(translation2d);
	}

	@Override
	public void refreshData() {
		this.refreshModulesData();
		this.refreshPigeon2Data();
	}

	private void refreshModulesData() {
		for (int i = 0; i < 4; i++) {
			var moduleMap = this.signalsMap.get(i);
			BaseStatusSignal.refreshAll(moduleMap.values().toArray(new BaseStatusSignal[] {}));
		}
	}

	private void refreshPigeon2Data() {
		BaseStatusSignal.refreshAll(this.pigeon2Signals.values().toArray(new BaseStatusSignal[] {}));
	}

	@Override
	public void disableMotors() {
		this.setSwerveState(new SwerveRequest.Idle());
	}

	@Override
	public void setStandardDeviationsForState(Matrix<N3, N1> stateStdDevs) {
		super.setStateStdDevs(stateStdDevs);
	}

	@Override
	public void addVisionMeasurement(VisionFieldPoseEstimate visionFieldPoseEstimate) {
		if (visionFieldPoseEstimate.getVisionMeasurementStdDevs() == null) {
			this.addVisionMeasurement(
					visionFieldPoseEstimate.getVisionRobotPoseMeters(),
					visionFieldPoseEstimate.getTimestampSeconds()
				);
		} else {
			this.addVisionMeasurement(
					visionFieldPoseEstimate.getVisionRobotPoseMeters(),
					visionFieldPoseEstimate.getTimestampSeconds(),
					visionFieldPoseEstimate.getVisionMeasurementStdDevs()
				);
		}
	}

	@Override
	public void forEachDriveMotor(Consumer<TalonFX> consumer) {
		for (var module : this.getModules()) {
			consumer.accept(module.getDriveMotor());
		}
	}

	@Override
	public void forEachSteerMotor(Consumer<TalonFX> consumer) {
		for (var module : this.getModules()) {
			consumer.accept(module.getSteerMotor());
		}
	}

}
