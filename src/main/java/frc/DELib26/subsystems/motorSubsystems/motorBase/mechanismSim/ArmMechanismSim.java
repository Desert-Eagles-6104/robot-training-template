package frc.DELib26.subsystems.motorSubsystems.motorBase.mechanismSim;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import frc.DELib26.subsystems.motorSubsystems.motorBase.IOs.SimTalonFXIO;
import frc.DELib26.subsystems.motorSubsystems.motorBase.MotorSubsystemConfiguration;

public class ArmMechanismSim extends MechanismSim<N2, N1, N2> {
	public static class ArmMechanismSimConfig extends MechanismSimConfig {

		public double armLengthMeters = Double.NaN;
		/**
		 * Angle offset (radians) such that {@code simAngle = mechAngle + angleOffsetRad}.
		 * Example: mech zero = vertical down (−Y) → {@code angleOffsetRad = -Math.PI/2}.
		 */
		public double angleOffsetRad = Double.NaN;

		public double armMinAngleRad = Double.NaN;
		public double armMaxAngleRad = Double.NaN;
		public double armStartingAngleRad = Double.NaN;


		/**
		 * Computes an "effective" arm length for WPILib's SingleJointedArmSim.
		 * <p>
		 * This is not the physical arm length. It is the length of an equivalent
		 * uniform rod (pivoted at one end) that produces similar gravity behavior
		 * for an arm whose center of mass is at a known distance from the pivot.
		 * <p>
		 * Intended for use when the real arm's mass distribution is non-uniform
		 * and the true moment of inertia is unknown.
		 *
		 * @param tipLengthFromPivotMeters Physical arm length from pivot to tip
		 * @param centerOfMassFromPivotMeters Distance from pivot to center of mass
		 * @return Effective arm length to pass to SingleJointedArmSim
		 */
		public static double getEffectiveArmLengthMeters(
			double tipLengthFromPivotMeters,
			double centerOfMassFromPivotMeters
		) {
			if (centerOfMassFromPivotMeters <= 0.0) {
				throw new IllegalArgumentException("centerOfMassFromPivotMeters must be > 0");
			}
			return (tipLengthFromPivotMeters * tipLengthFromPivotMeters)
					/ (2.0 * centerOfMassFromPivotMeters);
		}


		public ArmMechanismSimConfig withArmLengthMeters(double armLengthMeters) {
			this.armLengthMeters = armLengthMeters;
			return this;
		}

		/**
		 * Angle offset (radians) such that {@code simAngle = mechAngle + angleOffsetRad}.
		 * Example: mech zero = vertical down (−Y) → {@code angleOffsetRad = -Math.PI/2}.
		 */
		public ArmMechanismSimConfig withAngleOffsetRad(double angleOffsetRad) {
			this.angleOffsetRad = angleOffsetRad;
			return this;
		}

		public ArmMechanismSimConfig withArmMinAngleRad(double armMinAngleRad) {
			this.armMinAngleRad = armMinAngleRad;
			return this;
		}

		public ArmMechanismSimConfig withArmMaxAngleRad(double armMaxAngleRad) {
			this.armMaxAngleRad = armMaxAngleRad;
			return this;
		}

		public ArmMechanismSimConfig withArmStartingAngleRad(double armStartingAngleRad) {
			this.armStartingAngleRad = armStartingAngleRad;
			return this;
		}

		private ArmMechanismSimConfig applyLimitsFrom(MotorSubsystemConfiguration sc) {
			// Home is in your subsystem units -> convert to mech radians -> then to sim radians
			double homeMechRad = Units.degreesToRadians(sc.homePositionUnits * sc.conventionalPerSubsystemUnit);
			double homeSimRad  = homeMechRad + this.angleOffsetRad;

			TalonFXConfiguration masterCfg = sc.master.getTalonFXConfig();

			// Soft limits are in rotations (same unit as position signal).
			double revLimitRot = masterCfg.SoftwareLimitSwitch.ReverseSoftLimitThreshold;
			double fwdLimitRot = masterCfg.SoftwareLimitSwitch.ForwardSoftLimitThreshold;

			// rotations -> mech radians
			double revMechRad = Units.rotationsToRadians(revLimitRot);
			double fwdMechRad = Units.rotationsToRadians(fwdLimitRot);

			// mech -> sim
			double revSimRad = revMechRad + this.angleOffsetRad;
			double fwdSimRad = fwdMechRad + this.angleOffsetRad;

			// Ensure min <= max no matter what convention does
			this.armMinAngleRad = Math.min(revSimRad, fwdSimRad);
			this.armMaxAngleRad = Math.max(revSimRad, fwdSimRad);
			// Clamp start into range
			this.armStartingAngleRad = MathUtil.clamp(homeSimRad, this.armMinAngleRad, this.armMaxAngleRad);

			return this;
		}

		public ArmMechanismSimConfig(double momentOfInertia, double frictionVoltage) {
			super(momentOfInertia, frictionVoltage);
		}


		@Override
		public ArmMechanismSim create(
				MotorSubsystemConfiguration subsystemConfig,
				SimTalonFXIO masterIO,
				SimTalonFXIO[] followerIOs
			) {
			//if all are not set
			if (Double.isNaN(this.armStartingAngleRad) && Double.isNaN(this.armMinAngleRad) && Double.isNaN(this.armMaxAngleRad) ) {
				this.applyLimitsFrom(subsystemConfig);
			} else if (Double.isNaN(this.armStartingAngleRad) || Double.isNaN(this.armMinAngleRad) || Double.isNaN(this.armMaxAngleRad)) {
				throw new IllegalArgumentException("If any of armStartingAngleRad, armMinAngleRad, or armMaxAngleRad are set, then all must be set. Otherwise, leave all as NaN to auto-configure based on the subsystem configuration.");
			}

			return new ArmMechanismSim(
					this,
					subsystemConfig,
					masterIO,
					followerIOs
				);
		}
	}

	private final double angleOffsetRad;

	public ArmMechanismSim(
			ArmMechanismSimConfig mechanismSimConfig,
			MotorSubsystemConfiguration subsystemConfig,
			SimTalonFXIO masterIO,
			SimTalonFXIO[] followerIOs
		) {
		super(
				new SingleJointedArmSim(
						subsystemConfig.getCombinedMotor(),
						subsystemConfig.master.getTalonFXConfig().Feedback.SensorToMechanismRatio,
						mechanismSimConfig.momentOfInertia,
						mechanismSimConfig.armLengthMeters,
						mechanismSimConfig.armMinAngleRad,
						mechanismSimConfig.armMaxAngleRad,
						true, // simulateGravity
						mechanismSimConfig.armStartingAngleRad,
							0.001,
							0.001
				),
				mechanismSimConfig,
				masterIO,
				followerIOs);
		this.angleOffsetRad = mechanismSimConfig.angleOffsetRad;
		this.key = "Subsystems/" + subsystemConfig.subsystemName;
	}

	private SingleJointedArmSim getSim() {
		return (SingleJointedArmSim) this.sim;
	}

	private double rotorPerMech() {
		return this.masterIO.rotorPerMech();
	}

	@Override
	protected double getSimulatedRotorPositionRotations() {
		double mechAngleRad = this.getSim().getAngleRads() - angleOffsetRad;
		return Units.radiansToRotations(mechAngleRad) * this.rotorPerMech();
	}

	@Override
	protected double getSimulatedRotorVelocityRPS() {
		double mechVelRadPerSec = this.getSim().getVelocityRadPerSec();
		double mechVelRPS = Units.radiansToRotations(mechVelRadPerSec);
		return mechVelRPS * this.rotorPerMech();
	}

	@Override
	protected void setSimulatedRotorPositionRotations(double rotorRot) {
		double mechRot = rotorRot / this.rotorPerMech();
		double mechAngleRad = Units.rotationsToRadians(mechRot);
		this.getSim().setState(mechAngleRad + angleOffsetRad, this.getSim().getVelocityRadPerSec());
	}
}