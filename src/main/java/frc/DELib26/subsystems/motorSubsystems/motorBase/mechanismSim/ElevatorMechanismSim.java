package frc.DELib26.subsystems.motorSubsystems.motorBase.mechanismSim;

import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import frc.DELib26.subsystems.motorSubsystems.motorBase.IOs.SimTalonFXIO;
import frc.DELib26.subsystems.motorSubsystems.motorBase.MotorSubsystemConfiguration;


public class ElevatorMechanismSim extends MechanismSim<N2, N1, N2> {
	public static class ElevatorMechanismSimConfig extends MechanismSimConfig {
		public double gearing;
		public double carriageMassKg;
		public double drumRadiusMeters;

		public ElevatorMechanismSimConfig(double frictionVoltage) {
			super(frictionVoltage);
		}

		public ElevatorMechanismSimConfig withGearing(double gearing) {
			this.gearing = gearing;
			return this;
		}

		public ElevatorMechanismSimConfig withCarriageMassKg(double carriageMassKg) {
			this.carriageMassKg = carriageMassKg;
			return this;
		}

		public ElevatorMechanismSimConfig withDrumRadiusMeters(double drumRadiusMeters) {
			this.drumRadiusMeters = drumRadiusMeters;
			return this;
		}

		@Override
		public ElevatorMechanismSim create(
				MotorSubsystemConfiguration subsystemConfig,
				SimTalonFXIO masterIO,
				SimTalonFXIO[] followerIOs) {

			double rotorRotationsPerMeter = masterIO.rotorPerMech()
				* (1.0 / subsystemConfig.unitsPerMechRot)
				* (1.0 / subsystemConfig.conventionalPerSubsystemUnit);

			var softwareLimits = subsystemConfig.master.getTalonFXConfig().SoftwareLimitSwitch;
			double metersPerMechanismRotation = subsystemConfig.unitsPerMechRot
					* subsystemConfig.conventionalPerSubsystemUnit;
			double minHeightMeters = softwareLimits.ReverseSoftLimitThreshold * metersPerMechanismRotation;
			double maxHeightMeters = softwareLimits.ForwardSoftLimitThreshold * metersPerMechanismRotation;

			return new ElevatorMechanismSim(
					this,
					subsystemConfig,
					masterIO,
					followerIOs,
					rotorRotationsPerMeter,
					minHeightMeters,
					maxHeightMeters
				);
		}
	}

	private final double rotorRotationsPerMeter;

	public ElevatorMechanismSim(
			ElevatorMechanismSimConfig mechanismSimConfig,
			MotorSubsystemConfiguration subsystemConfig,
			SimTalonFXIO masterIO,
			SimTalonFXIO[] followerIOs,
			double rotationsPerMeter,
			double minHeightMeters,
			double maxHeightMeters
		) {
		super(
				new ElevatorSim(
						subsystemConfig.getCombinedMotor(),
						mechanismSimConfig.gearing,
						mechanismSimConfig.carriageMassKg,
						mechanismSimConfig.drumRadiusMeters,
						minHeightMeters,
						maxHeightMeters,
						true,
						0.0,
						0.001,
						0.001),
				mechanismSimConfig,
				masterIO,
				followerIOs);
		this.rotorRotationsPerMeter = rotationsPerMeter;
		this.key = "Subsystems/" + subsystemConfig.subsystemName;
	}

	private ElevatorSim getSim() {
		return (ElevatorSim) this.sim;
	}

	@Override
	protected double getSimulatedRotorPositionRotations() {
		return this.rotorRotationsPerMeter * this.getSim().getPositionMeters();
	}

	@Override
	protected double getSimulatedRotorVelocityRPS() {
		return this.rotorRotationsPerMeter * this.getSim().getVelocityMetersPerSecond();
	}
}
