package frc.DELib26.subsystems.motorSubsystems.motorBase.IOs;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.HardwareLimitSwitchConfigs;
import com.ctre.phoenix6.configs.SoftwareLimitSwitchConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.MotionMagicVelocityVoltage;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularAcceleration;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;
import frc.DELib26.motors.MotorConstants;
import frc.DELib26.motors.MotorIO;
import frc.DELib26.subsystems.motorSubsystems.motorBase.MotorSubsystemConfiguration;
import frc.DELib26.util.PhoneixUtil;

public class TalonFXIO implements MotorIO {
	// 20 ms * 5 = 100ms - 10hz
	private static final int UPDATE_FREQUENCY_INTERVAL = 5;
	// i maby will replace it with log path
	protected final MotorSubsystemConfiguration subsystemConfig;
	protected final MotorConstants motorConstants;
	protected final TalonFX motor;

	// private final double rotationsPerPositionUnit;

	// Requests
	private final MotionMagicVoltage motionMagicVoltageRequest = new MotionMagicVoltage(0).withSlot(0);
	private final PositionVoltage positionVoltageRequest = new PositionVoltage(0).withSlot(0);
	private final DutyCycleOut dutyCycleRequest = new DutyCycleOut(0);
	private final VoltageOut sysIdControlRequest = new VoltageOut(0);
	private final MotionMagicVelocityVoltage motionMagicVelocityRequest = new MotionMagicVelocityVoltage(0).withSlot(0);
	private final VelocityVoltage velocityVoltageRequest = new VelocityVoltage(0);

	private int pidSlot;

	// Signals
	protected final StatusSignal<Angle> positionSignal;// its in mec rotations
	protected final StatusSignal<Angle> rawRotorPositionSignal;
	protected final StatusSignal<AngularVelocity> velocitySignal;
	protected final StatusSignal<AngularAcceleration> accelerationSignal;
	protected final StatusSignal<Current> supplyCurrentSignal;
	protected final StatusSignal<Current> statorCurrentSignal;
	protected final StatusSignal<Double> closedLoopErrorSignal;
	protected final StatusSignal<Voltage> appliedVoltageSignal;
	protected final StatusSignal<Temperature> temperatureSignal;

	private final BaseStatusSignal[] fastSignals;
	private final BaseStatusSignal[] slowSignals;

	private int updateCounter = 0;

	public TalonFXIO(MotorSubsystemConfiguration subsystemConfig, MotorConstants motorConstants, TalonFX motor) {
		this.subsystemConfig = subsystemConfig;
		this.motorConstants = motorConstants;
		this.motor = motor;

		this.positionSignal = this.motor.getPosition();
		this.rawRotorPositionSignal = this.motor.getRotorPosition();
		this.velocitySignal = this.motor.getVelocity();
		this.accelerationSignal = this.motor.getAcceleration();
		this.supplyCurrentSignal = this.motor.getSupplyCurrent();
		this.statorCurrentSignal = this.motor.getStatorCurrent();
		this.appliedVoltageSignal = this.motor.getMotorVoltage();
		this.closedLoopErrorSignal = this.motor.getClosedLoopError();
		this.temperatureSignal = this.motor.getDeviceTemp();
		// i dont want to refresh the temperature signal as much as the others because
		// it changes more slowly and refreshing it takes time
		this.fastSignals = new BaseStatusSignal[] {
			this.closedLoopErrorSignal,
			this.positionSignal,
			this.rawRotorPositionSignal,
			this.velocitySignal,
			this.accelerationSignal,
			this.appliedVoltageSignal,
			this.supplyCurrentSignal,
			this.statorCurrentSignal
		};
		this.slowSignals = new BaseStatusSignal[] {
			this.temperatureSignal
		};

		BaseStatusSignal.setUpdateFrequencyForAll(50, this.fastSignals);
		BaseStatusSignal.setUpdateFrequencyForAll(10, this.slowSignals);

		this.motor.optimizeBusUtilization();
		this.setPIDSlot(0);
	}

	@Override
	public void updateInputs(MotorIOInputs inputs) {
		BaseStatusSignal.refreshAll(this.fastSignals);
		if (this.updateCounter++ % UPDATE_FREQUENCY_INTERVAL == 0) {
			BaseStatusSignal.refreshAll(this.slowSignals);
		}
		inputs.mechanismPosition = this.positionSignal.getValue();
		inputs.positionUnits = this.toUnits(inputs.mechanismPosition);
		inputs.velocityUnitsPerSec = this.velocitySignal.getValue().in(Units.RotationsPerSecond);
		inputs.mechanismVelocityRotPerSec = this.velocitySignal.getValueAsDouble();
		inputs.accelerationUnitsPerSecPerSec = this.toUnits(this.accelerationSignal.getValueAsDouble());
		inputs.supplyCurrent = this.supplyCurrentSignal.getValue();
		inputs.statorCurrent = this.statorCurrentSignal.getValue();
		inputs.closedLoopError = this.toUnits(this.closedLoopErrorSignal.getValueAsDouble());
		inputs.appliedVoltage = this.appliedVoltageSignal.getValue();
		inputs.temperature = Units.Kelvin.of(this.temperatureSignal.getValueAsDouble());
	}

	@Override
	public void disableMotors() {
		this.motor.disable();
	}

	@Override
	public void resetPosition(double positionUnits) {
		this.motor.setPosition(this.toRotations(positionUnits));
	}

	@Override
	public void setPositionSetpoint(double positionUnits) {
		this.motor.setControl(
				this.positionVoltageRequest.withPosition(this.toRotations(positionUnits)).withSlot(this.pidSlot)
			);
	}

	@Override
	public void setMotionMagicPositionSetpoint(double positionUnits) {
		this.motor.setControl(
				this.motionMagicVoltageRequest.withPosition(this.toRotations(positionUnits)).withSlot(this.pidSlot)
			);
	}

	@Override
	public void setMotionMagicVelocity(double velocityUnits) {
		this.motor.setControl(
				this.motionMagicVelocityRequest.withVelocity(this.toRotations(velocityUnits)).withSlot(this.pidSlot)
			);
	}

	@Override
	public void setVelocity(double velocityUnits) {
		this.motor.setControl(
				this.velocityVoltageRequest.withVelocity(this.toRotations(velocityUnits)).withSlot(this.pidSlot)
			);
	}

	@Override
	public void setPercentOutput(double precent, boolean ignoreSoftLimits) {
		this.motor.setControl(
			this.dutyCycleRequest.withOutput(precent)
			.withIgnoreSoftwareLimits(ignoreSoftLimits)
		);
	}

	@Override
	public void setPIDSlot(int newSlot) {
		if (newSlot >= 0 && newSlot <= 2) {
			this.pidSlot = newSlot;
		} else {
			System.err.println("PID SLOT [" + newSlot + "] is out of bound for [0..2]");
		}
	}

	@Override
	public void runCharacterization(Voltage volts) {
		this.motor.setControl(this.sysIdControlRequest.withOutput(volts.in(Units.Volts)));
	}

	protected double toUnits(Angle position) {
		return this.toUnits(position.in(Units.Rotations));
	}

	protected double toUnits(double positionRotations) {
		return positionRotations * this.subsystemConfig.unitsPerMechRot;
	}

	protected double toRotations(double units) {
		return units / this.subsystemConfig.unitsPerMechRot;
	}

	@Override
	public void controlForwardSoftLimit(boolean enableSoftLimit, double forwardSoftLimitThreshold) {
		PhoneixUtil.applyConfiguration(
				this.motor,
				new SoftwareLimitSwitchConfigs()
						.withForwardSoftLimitEnable(enableSoftLimit) // The initial value given to the configuration.
						.withForwardSoftLimitThreshold(forwardSoftLimitThreshold));
	}

	@Override
	public void controlReverseSoftLimit(boolean enableSoftLimit, double reverseSoftLimitThreshold) {
		PhoneixUtil.applyConfiguration(
				this.motor,
				new SoftwareLimitSwitchConfigs()
						.withReverseSoftLimitEnable(enableSoftLimit) // The initial value given to the configuration.
						.withReverseSoftLimitThreshold(reverseSoftLimitThreshold));
	}

	@Override
	public void setEnableAutosetPositionValue(boolean forward, boolean reverse) {
		PhoneixUtil.applyConfiguration(this.motor,
				new HardwareLimitSwitchConfigs()
						.withForwardLimitAutosetPositionEnable(forward)
						.withReverseLimitAutosetPositionEnable(reverse)
						.withForwardLimitEnable(forward)
						.withReverseLimitEnable(reverse));
	}

	@Override
	public void setNeutralMode(NeutralModeValue mode) {
		this.motor.setNeutralMode(mode);
	}

	@Override
	public void applyConfig(TalonFXConfiguration configs) {
		PhoneixUtil.applyConfiguration(this.motor, configs);
	}

}
