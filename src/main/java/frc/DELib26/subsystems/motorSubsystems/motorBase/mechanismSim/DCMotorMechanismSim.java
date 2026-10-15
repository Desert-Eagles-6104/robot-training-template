package frc.DELib26.subsystems.motorSubsystems.motorBase.mechanismSim;

import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import frc.DELib26.subsystems.motorSubsystems.motorBase.IOs.SimTalonFXIO;
import frc.DELib26.subsystems.motorSubsystems.motorBase.MotorSubsystemConfiguration;
import frc.DELib26.util.Conversions;
/**
 * Simulation helper for a mechanism driven by a DC motor with both position and velocity states.
 *
 * <p>This class models a second-order rotary system (states: {@code [position, velocity]ᵀ})
 * using a DC-motor plant (e.g., from {@link edu.wpi.first.math.system.plant.LinearSystemId#createDCMotorSystem}).
 * Use this when you care about angle tracking (turn-to-position, turret, etc.), not just speed.
 *
 * <p><b>How this differs from {@link edu.wpi.first.wpilibj.simulation.FlywheelSim}:</b>
 * <ul>
 *   <li>{@code DCMotorMechanismSim} tracks <i>position and velocity</i> (2 states).</li>
 *   <li>{@code FlywheelSim} tracks <i>velocity only</i> (1 state), which is ideal for shooters and wheels
 *       where absolute angle is irrelevant.</li>
 * </ul>
 *
 * <p><b>Units & parameters:</b>
 * <ul>
 *   <li>Angle in radians (position), angular velocity in radians/second.</li>
 *   <li>Moment of inertia J in kg·m² (about the output axis).</li>
 *   <li>Gearing as output/input (e.g., 10:1 reduction → {@code gearing = 10}).</li>
 * </ul>
 *
 * <p><b>Notes:</b>
 * <ul>
 *   <li>Gravity torque is not inherently modeled; add a gravity feedforward if needed
 *       (for arms use {@link edu.wpi.first.math.system.plant.LinearSystemId#createSingleJointedArmSystem}).</li>
 *   <li>For speed-only mechanisms, prefer {@link edu.wpi.first.wpilibj.simulation.FlywheelSim}.</li>
 * </ul>
 *
 * @see edu.wpifirst.wpilibj.simulation.DCMotorSim
 * @see edu.wpifirst.wpilibj.simulation.FlywheelSim
 * @see edu.wpifirst.math.system.plant.LinearSystemId
 */
public class DCMotorMechanismSim extends MechanismSim<N2, N1, N2> {
	public static class DCMotorMechanismSimConfig extends MechanismSimConfig {

		public DCMotorMechanismSimConfig(double momentOfInertia, double frictionVoltage) {
			super(momentOfInertia, frictionVoltage);
		}

		@Override
		public DCMotorMechanismSim create(
				MotorSubsystemConfiguration subsystemConfig,
				SimTalonFXIO masterIO,
				SimTalonFXIO[] followerIOs
		) {
			return new DCMotorMechanismSim(
					this,
					subsystemConfig,
					masterIO,
					followerIOs
				);
		}
	}

	public DCMotorMechanismSim(
			DCMotorMechanismSimConfig mechanismSimConfig,
			MotorSubsystemConfiguration subsystemConfig,
			SimTalonFXIO masterIO,
			SimTalonFXIO[] followerIOs) {
		super(
				new DCMotorSim(
						LinearSystemId.createDCMotorSystem(
							subsystemConfig.getCombinedMotor(),
							mechanismSimConfig.momentOfInertia,
							subsystemConfig.master.getTalonFXConfig().Feedback.SensorToMechanismRatio
						),
						subsystemConfig.getCombinedMotor(),
						0.001,
						0.001),
				mechanismSimConfig,
				masterIO,
				followerIOs);
		this.key = "Subsystems/" + subsystemConfig.subsystemName;
	}

	private DCMotorSim getSim() {
		return (DCMotorSim) this.sim;
	}

	private double rotorPerMech() {
		return this.masterIO.rotorPerMech();
	}

	@Override
	protected double getSimulatedRotorPositionRotations() {
		return this.getSim().getAngularPositionRotations() * this.rotorPerMech();
	}

	@Override
	protected double getSimulatedRotorVelocityRPS() {
		double mechVelRPS = this.getSim().getAngularVelocityRadPerSec() / Conversions.RADIANS_PER_ROTATION;
		return mechVelRPS * this.rotorPerMech();
	}

	@Override
	protected void setSimulatedRotorVelocityRPS(double rotorVelocityRPS) {
		this.getSim().setAngularVelocity(
			rotorVelocityRPS / this.rotorPerMech() * Conversions.RADIANS_PER_ROTATION
		);
	}
}
