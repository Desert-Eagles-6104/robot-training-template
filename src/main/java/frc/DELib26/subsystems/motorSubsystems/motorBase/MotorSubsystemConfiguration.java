package frc.DELib26.subsystems.motorSubsystems.motorBase;

import edu.wpi.first.math.system.plant.DCMotor;
import frc.DELib26.motors.MasterMotorConstants;
import frc.DELib26.motors.MotorConstants;
import frc.DELib26.subsystems.motorSubsystems.motorBase.mechanismSim.MechanismSimConfig;

import java.util.Optional;

public class MotorSubsystemConfiguration {

	/** The name of this subsystem. */
	public String subsystemName = "";

	/** Master motor configuration (required). */
	public MasterMotorConstants master;

	/** Slave motor configurations (optional). */
	public Optional<MotorConstants[]> slaves = Optional.empty();

	/**
	 * how many units per (one) mech rotation
	 * Ratio of units to rotor for this talon. units * by this ratio should be the
	 * rotors.
	 * >1 is reduction
	 */
	public double unitsPerMechRot = 1.0;

	/**
	 * Scale to convert **one subsystem unit** to the chosen conventional unit
	 * (meters for linear, degrees for angular).
	 * Examples:
	 *  - If subsystem unit is mm → 0.001 m/mm
	 *  - If subsystem unit is cm → 0.01 m/cm
	 *  - If subsystem unit is rad → 57.2957795 deg/rad
	 *  - If subsystem unit is already m or deg → 1.0
	 */
	public double conventionalPerSubsystemUnit = 1.0;

	public enum MechanismType {
		LINEAR,
		ANGULAR
	}

	public MechanismType mechanismType;

	public double allowableErrorUnits;

	public double homePositionUnits = 0.0;

	public MechanismSimConfig mechanismSimConfig;

	public MotorSubsystemConfiguration withName(String name) {
		this.subsystemName = name;
		return this;
	}

	public MotorSubsystemConfiguration withMaster(MasterMotorConstants master) {
		this.master = master;
		return this;
	}

	public MotorSubsystemConfiguration withSlaves(MotorConstants... slaves) {
		this.slaves = Optional.of(slaves);
		return this;
	}

	public MotorSubsystemConfiguration withRotationsPerPositionUnit(double rotationsPerPositionUnit) {
		this.unitsPerMechRot = rotationsPerPositionUnit;
		return this;
	}

	public MotorSubsystemConfiguration withAllowableError(double allowableError) {
		this.allowableErrorUnits = allowableError;
		return this;
	}

	public MotorSubsystemConfiguration withHomePosition(double homePosition) {
		this.homePositionUnits = homePosition;
		return this;
	}

	public MotorSubsystemConfiguration withMechanismSimConfig(MechanismSimConfig mechanismSimConfig) {
		this.mechanismSimConfig = mechanismSimConfig;
		return this;
	}

	public MotorSubsystemConfiguration withConventionalPerSubsystemUnit(double conventionalPerSubsystemUnit) {
		this.conventionalPerSubsystemUnit = conventionalPerSubsystemUnit;
		return this;
	}

	public MotorSubsystemConfiguration withMechanismType(MechanismType mechanismType) {
		this.mechanismType = mechanismType;
		return this;
	}

	public DCMotor getCombinedMotor() {
		DCMotor masterMotor = this.master.getMotorSimulationSetting();
		if (this.slaves.isPresent()) {
			for (MotorConstants slave : this.slaves.get()) {
				DCMotor slaveMotor = slave.getMotorSimulationSetting();
				if (!isMotorEquals(masterMotor, slaveMotor)) {
					throw new IllegalArgumentException("All motors in a subsystem must be of the same type.");
				}
			}
		}
		return new DCMotor(
				masterMotor.nominalVoltageVolts,
				masterMotor.stallTorqueNewtonMeters,
				masterMotor.stallCurrentAmps,
				masterMotor.freeCurrentAmps,
				masterMotor.freeSpeedRadPerSec,
				this.getTotalMotors()
			);
	}

	public int getTotalMotors() {
		return 1 + this.slaves.map(s -> s.length).orElse(0);
	}

	public static boolean isMotorEquals(DCMotor motor, DCMotor otherMotor) {
		return motor.nominalVoltageVolts == otherMotor.nominalVoltageVolts
				&& motor.stallTorqueNewtonMeters == otherMotor.stallTorqueNewtonMeters
				&& motor.stallCurrentAmps == otherMotor.stallCurrentAmps
				&& motor.freeCurrentAmps == otherMotor.freeCurrentAmps
				&& motor.freeSpeedRadPerSec == otherMotor.freeSpeedRadPerSec
				&& motor.rOhms == otherMotor.rOhms
				&& motor.KvRadPerSecPerVolt == otherMotor.KvRadPerSecPerVolt
				&& motor.KtNMPerAmp == otherMotor.KtNMPerAmp;
	}

}
