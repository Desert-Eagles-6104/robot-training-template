package frc.DELib26.subsystems.motorSubsystems.motorBase;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.DELib26.motors.MotorConstants;
import frc.DELib26.motors.MotorFactory;
import frc.DELib26.motors.MotorIO;
import frc.DELib26.motors.MotorIOInputsAutoLogged;
import frc.DELib26.subsystems.motorSubsystems.motorBase.IOs.SimTalonFXIO;
import frc.DELib26.subsystems.motorSubsystems.motorBase.IOs.TalonFXIO;
import frc.DELib26.subsystems.motorSubsystems.motorBase.mechanismSim.MechanismSim;
import frc.DELib26.util.RunMode;
import org.littletonrobotics.junction.Logger;

import java.util.Arrays;
import java.util.Optional;
import java.util.function.Consumer;

public class MotorSubsystemTalon extends SubsystemBase {

	public MotorSubsystemConfiguration configuration;

	private double positionSetpoint;
	private double velocitySetpoint;

	private final MotorIOInputsAutoLogged masterInputs;
	private final MotorIO masterIO;

	private final MotorIOInputsAutoLogged[] followerInputs;
	private final MotorIO[] followerIOs;

	private final Optional<MechanismSim<?, ?, ?>> mechanismSim;

	public MotorSubsystemTalon(MotorSubsystemConfiguration configuration) {
		this(
			configuration,
			buildMasterIO(configuration),
			buildFollowerIOs(configuration)
		);
	}

	public MotorSubsystemTalon(
			MotorSubsystemConfiguration configuration,
			MotorIO masterIO,
			MotorIO[] followerIOs) {
		this.configuration = configuration;

		this.masterIO = masterIO;
		this.masterInputs = new MotorIOInputsAutoLogged();

		this.followerIOs = followerIOs;
		this.followerInputs = new MotorIOInputsAutoLogged[this.followerIOs.length];

		for (int i = 0; i < this.followerInputs.length; i++) {
			this.followerInputs[i] = new MotorIOInputsAutoLogged();
		}

		if (RunMode.isSim()) {
			// java does'nt like arr casting
			SimTalonFXIO[] arr = new SimTalonFXIO[this.followerIOs.length];
			for (int i = 0; i < this.followerIOs.length; i++) {
				arr[i] = (SimTalonFXIO) this.followerIOs[i];
			}
			this.mechanismSim = Optional.of(
					this.configuration.mechanismSimConfig.create(
						this.configuration,
						(SimTalonFXIO) this.masterIO,
						arr
					)
				);
		} else {
			// you cant pose reset Until the simulator started ticking so in SIM we can't do
			// it over here. And there also is no need to reset in sim
			this.resetToHomePosition();
			this.mechanismSim = Optional.empty();
		}

		super.setName(this.configuration.subsystemName);
	}

	private static MotorIO createIO(
			MotorSubsystemConfiguration configuration,
			MotorConstants motorConstants,
			TalonFX motor) {
		if (RunMode.isSim()) {
			return new SimTalonFXIO(configuration, motorConstants, motor);
		} else {
			return new TalonFXIO(configuration, motorConstants, motor);
		}
	}

	public static MotorIO buildMasterIO(MotorSubsystemConfiguration configuration) {
		return createIO(
				configuration,
				configuration.master,
				MotorFactory.createTalonFX(configuration.master));
	}

	public static MotorIO[] buildFollowerIOs(MotorSubsystemConfiguration configuration) {
		final int followerCount = configuration.slaves.map(a -> a.length).orElse(0);

		// Dont return a interface array brcuse it will cause issues
		if (followerCount == 0) {
			if (RunMode.isSim()) {
				return new SimTalonFXIO[0];
			} else {
				return new TalonFXIO[0];
			}
		}

		final MotorConstants[] slaves = configuration.slaves.orElseThrow();
		final TalonFX[] slaveTalons = MotorFactory.createSlaveTalonsAndApplyConfig(slaves, configuration.master);

		MotorIO[] ios = new MotorIO[followerCount];
		for (int i = 0; i < followerCount; i++) {
			ios[i] = createIO(configuration, slaves[i], slaveTalons[i]);
		}

		return ios;
	}

	public Optional<MechanismSim<?, ?, ?>> getMechanismSim() {
		return this.mechanismSim;
	}

	@Override
	public void periodic() {
		this.masterIO.updateInputs(this.masterInputs);
		for (int i = 0; i < this.followerIOs.length; i++) {
			this.followerIOs[i].updateInputs(this.followerInputs[i]);
		}
	}

	public final void logTelemetry() {
		Logger.processInputs(this.getKey() + "/Inputs", this.masterInputs);

		var command = this.getCurrentCommand();
		// We log the name of the command, or null if there is no command (we want to
		// know when there is no command)
		String commandName = command != null ? command.getName() : "None";
		Logger.recordOutput(this.getKey() + "/Command", commandName);

	}

	public void logExtendedTelemetry() {
		this.logTelemetry();
		for (int i = 0; i < this.followerIOs.length; i++) {
			Logger.processInputs(this.getKey() + "/Follower" + i + "Inputs", this.followerInputs[i]);
		}
	}

	public void resetToHomePosition() {
		this.resetPosition(this.configuration.homePositionUnits);
	}

	public void disableMotors() {
		this.masterIO.disableMotors();
	}

	public void runCharacterization(Voltage volts) {
		this.masterIO.runCharacterization(volts);
	}

	public void setMotionMagicPosition(double positionUnits) {
		this.positionSetpoint = positionUnits;
		this.masterIO.setMotionMagicPositionSetpoint(positionUnits);
	}

	public void setPosition(double positionUnits) {
		this.positionSetpoint = positionUnits;
		this.masterIO.setPositionSetpoint(positionUnits);
	}

	public void controlForwardSoftLimit(boolean enableSoftLimit) {
		double forwardSoftLimitThreshold = this.configuration.master
				.getTalonFXConfig().SoftwareLimitSwitch.ForwardSoftLimitThreshold;

		this.masterIO.controlForwardSoftLimit(enableSoftLimit, forwardSoftLimitThreshold);
	}

	public void controlReverseSoftLimit(boolean enableSoftLimit) {
		double reverseSoftLimitThreshold = this.configuration.master
				.getTalonFXConfig().SoftwareLimitSwitch.ReverseSoftLimitThreshold;
		this.masterIO.controlReverseSoftLimit(enableSoftLimit, reverseSoftLimitThreshold);
	}

	public void setPercentOutput(double precent, boolean ignoreSoftLimits) {
		this.masterIO.setPercentOutput(precent, ignoreSoftLimits);
	}

	public void setConfig(TalonFXConfiguration configs) {
		this.masterIO.applyConfig(configs);
	}

	/*
	 * Get the current position of the motor from the positionSignal
	 */
	public double getPosition() {
		return this.masterInputs.positionUnits;
	}

	/*
	 * Get the current position of the motor in conventional units (meters or
	 * degrees)
	 */
	public double getPositionInConventionalUnits() {
		return this.masterInputs.positionUnits * this.configuration.conventionalPerSubsystemUnit;
	}

	public void resetPosition(double positionUnits) {
		this.masterIO.resetPosition(positionUnits);
	}

	public boolean isAtPositionSetpoint() {
		return this.isAtPositionSetpoint(this.configuration.allowableErrorUnits);
	}

	public boolean isAtPositionSetpoint(double allowableErrorUnits) {
		return Math.abs(this.getPositionError()) < allowableErrorUnits;
	}

	public boolean isAtVelocitySetpoint() {
		return this.isAtVelocitySetpoint(this.configuration.allowableErrorUnits);
	}

	public boolean isAtVelocitySetpoint(double allowableErrorUnits) {
		return Math.abs(this.getVelocityError()) < allowableErrorUnits;
	}

	public MotorIO.MotorIOInputs getMasterInputs() {
		return this.masterInputs;
	}

	public double getPositionError() {
		return this.positionSetpoint - this.getPosition();
	}

	public double getPositionSetpoint() {
		return this.positionSetpoint;
	}

	public double getVelocityError() {
		return this.velocitySetpoint - this.getVelocity();
	}

	public double getVelocity() {
		double velocity = this.masterInputs.mechanismVelocityRotPerSec;
		for (MotorIOInputsAutoLogged followerInput : this.followerInputs) {
			velocity += followerInput.mechanismVelocityRotPerSec;
		}
		return velocity / (1 + this.followerInputs.length);
	}

	public double getVelocityInConventionalUnitsPerSecond() {
		return this.masterInputs.velocityUnitsPerSec * this.configuration.conventionalPerSubsystemUnit;
	}

	public Current getStatorCurrent() {
		return this.masterInputs.statorCurrent;
	}

	public void changeNeutralMode(NeutralModeValue mode) {
		this.forEachIO(io -> io.setNeutralMode(mode));
	}

	private void forEachIO(Consumer<MotorIO> action) {
		action.accept(this.masterIO);
		Arrays.stream(this.followerIOs).forEach(action);
	}

	public void setMotionMagicVelocity(double velocity) {
		this.masterIO.setMotionMagicVelocity(velocity);
	}

	public void setVelocity(double velocity) {
		this.velocitySetpoint = velocity;
		this.masterIO.setVelocity(velocity);
	}

	public void setPIDSlot(int slot) {
		this.masterIO.setPIDSlot(slot);
	}

	public String getKey() {
		return "Subsystems/" + this.getName();
	}

	// API
}
