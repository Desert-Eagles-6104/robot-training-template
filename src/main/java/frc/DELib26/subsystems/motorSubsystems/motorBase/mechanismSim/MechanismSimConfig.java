package frc.DELib26.subsystems.motorSubsystems.motorBase.mechanismSim;

import frc.DELib26.subsystems.motorSubsystems.motorBase.IOs.SimTalonFXIO;
import frc.DELib26.subsystems.motorSubsystems.motorBase.MotorSubsystemConfiguration;

public abstract class MechanismSimConfig {

	public double momentOfInertia;

	public double frictionVoltage;
	/**
	 * The period at which the simulation is updated, in seconds.
	 * The slower the period, the more unstable the sim becomes. The faster the period,
	 * the more CPU it uses but sim is more realistic.
	 */
	public final double simPeriodSec = 0.005;

	protected MechanismSimConfig(double momentOfInertia, double frictionVoltage) {
		this.momentOfInertia = momentOfInertia;
		this.frictionVoltage = frictionVoltage;
	}

	protected MechanismSimConfig(double frictionVoltage) {
		this.frictionVoltage = frictionVoltage;
	}

	public abstract MechanismSim<?, ?, ?> create(
		MotorSubsystemConfiguration subsystemConfig,
		SimTalonFXIO masterIO,
		SimTalonFXIO[] followerIOs
	);

}
