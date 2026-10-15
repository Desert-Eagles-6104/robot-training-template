package frc.DELib26.subsystems.motorSubsystems.motorBase.mechanismSim;

import edu.wpi.first.math.Num;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.simulation.LinearSystemSim;
import frc.DELib26.subsystems.motorSubsystems.motorBase.IOs.SimTalonFXIO;
import frc.DELib26.util.TimestampDeltaCalculator;
import org.ironmaple.simulation.SimulatedArena;
import org.ironmaple.simulation.SimulatedArena.Simulatable;
import org.ironmaple.simulation.motorsims.SimulatedBattery;
import org.littletonrobotics.junction.Logger;

import java.util.Optional;
import java.util.concurrent.atomic.AtomicReference;

/**
 * The `MechanismSim` class provides a simulation framework for mechanisms that
 * are driven
 * by one or more motors (e.g., an arm, elevator, wrist, roller). It uses a
 * {@link LinearSystemSim}
 * (state-space plant) to evolve the mechanism state in simulation based on the
 * voltage that the
 * motor controller *would* apply, which is retrieved from Phoenix 6's simulated
 * TalonFX via
 * {@code getSimState().getMotorVoltage()}.
 *
 * <h2>Why this exists</h2>
 * In simulation, there is no real encoder. CTRE’s simulated firmware will still
 * compute an
 * output duty cycle/voltage from your control requests (PercentOutput, Voltage,
 * Position/Velocity
 * closed-loop, Motion Magic, etc.). However, the controller needs sensor
 * feedback to close the loop.
 * This class:
 * <ol>
 * <li>Reads the controller’s <em>commanded voltage</em> (what it would put on
 * the motor terminals).</li>
 * <li>Feeds that into a physics model ({@link LinearSystemSim}) to integrate
 * position/velocity.</li>
 * <li>Writes the simulated <em>rotor</em> position/velocity back to the Talon
 * via
 * {@code setRawRotorPosition(...)} and {@code setRotorVelocity(...)}.</li>
 * </ol>
 *
 * <h2>Units & conventions</h2>
 * <ul>
 * <li><b>Voltages:</b> Volts (V). The supply voltage is taken from
 * {@link edu.wpi.first.wpilibj.RobotController#getBatteryVoltage()} and pushed
 * into the Talon sim.</li>
 * <li><b>Rotor signals:</b> The setters {@code setRawRotorPosition} and
 * {@code setRotorVelocity} are in
 * <i>rotor</i> units (rotations, rotations-per-second). If you configured
 * {@code FeedbackConfigs.withSensorToMechanismRatio(G)}, you must convert your
 * mechanism state into rotor
 * units before writing these values.</li>
 * <li><b>Mechanism state:</b> Your subclass decides whether to maintain state
 * in mechanism units
 * and convert, or track rotor units directly. See
 * {@link #getSimulatedRotorPositionRotations()} and
 * {@link #getSimulatedRotorVelocityRPS()}.</li>
 * </ul>
 *
 * <h2>Timing model</h2>
 * A {@link Notifier} ticks this sim at a fixed period (default 5 ms = 200 Hz).
 * Faster rates are more stable
 * and realistic for closed-loop control, but use more CPU. You can tweak the
 * period via the call to
 * {@code startPeriodic(...)} in the constructor.
 *
 * <h2>Followers</h2>
 * Any follower Talons supplied in {@code followerIOs} will receive the same
 * simulated rotor position/velocity
 * as the master. This mirrors typical FRC usage where followers share the same
 * sensor/shaft state.
 *
 * <h2>Subclassing</h2>
 * Implement {@link #getSimulatedRotorPositionRotations()} and
 * {@link #getSimulatedRotorVelocityRPS()} to return
 * the current <em>rotor</em> position/velocity derived from your plant.
 * <ul>
 * <li>If your plant tracks <em>mechanism</em> angle/speed, multiply by
 * {@code sensorPerMechanismRatio}
 * (i.e., gear ratio) to convert to rotor units.</li>
 * </ul>
 *
 * <h2>Threading</h2>
 * The {@link SimulatedArena} runs on its own real-time thread. Do not block in
 * {@link #simulationSubTick(int subTick)}.
 * Keep state shared between threads either atomic or confined; this class uses
 * {@link AtomicReference}
 * for the last-computed values. If you log or access additional shared state,
 * use appropriate concurrency hygiene.
 *
 * <h2>Key Phoenix 6 interactions</h2>
 * <ul>
 * <li>{@code simState.getMotorVoltage()} is the commanded terminal voltage from
 * Phoenix’s internal loop
 * (duty × bus V, or the result of PID in closed-loop modes).</li>
 * <li>{@code setRawRotorPosition(...)} and {@code setRotorVelocity(...)} feed
 * the Talon’s <em>simulated encoder</em>.</li>
 * <li>Setting {@code simState.setSupplyVoltage(busV)} lets the Talon scale
 * duty-cycle to volts realistically.</li>
 * </ul>
 *
 * @param <S> Number of <b>states</b> in the state-space system (e.g., position
 *            & velocity → 2).
 * @param <I> Number of <b>inputs</b> (typically 1 for a single motor voltage).
 * @param <O> Number of <b>outputs</b> you expose from the plant (often 1).
 */
public abstract class MechanismSim<S extends Num, I extends Num, O extends Num> implements Simulatable {
	/**
	 * The state-space plant that evolves the mechanism based on applied input
	 * voltage.
	 */
	protected final LinearSystemSim<S, I, O> sim;

	/**
	 * The master TalonFX I/O wrapper for reading commanded voltage and writing
	 * simulated sensor values.
	 */
	protected final SimTalonFXIO masterIO;

	/**
	 * Any follower Talons that should mirror the master’s simulated rotor
	 * position/velocity.
	 */
	protected final SimTalonFXIO[] followerIOs;

	/**
	 * Last computed rotor velocity (RPS). Used to compute acceleration.
	 */
	private Double lastRotorVelocityRPS = 0.0;

	private final AtomicReference<Optional<Double>> manualRotorPositionOverride = new AtomicReference<>(
			Optional.empty());
	private final AtomicReference<Optional<Double>> manualRotorVelocityOverride = new AtomicReference<>(
			Optional.empty());

	/**
	 * Effective friction voltage (V) for the simple Coulomb friction model.
	 */
	private double frictionVoltage;

	/**
	 * A short, human-friendly key used for logging.
	 */
	protected String key = "Subsystems/MechanismSim";

	protected final TimestampDeltaCalculator timestampDeltaCalculator;

	/**
	 * Constructs a new mechanism simulator.
	 *
	 * @param sim         The state-space plant representing the mechanism dynamics.
	 * @param masterIO    The master TalonFX I/O (provides sim state accessors).
	 * @param followerIOs Zero or more follower TalonFX I/Os that mirror the
	 *                    master’s rotor state.
	 *
	 *                    <p>
	 *                    <b>Timing:</b> Starts a {@link Notifier} at a set period.
	 *                    Lower periods increase CPU but
	 *                    improve closed-loop fidelity. You may edit the period in
	 *                    the constructor as needed.
	 */
	public MechanismSim(
			LinearSystemSim<S, I, O> sim,
			MechanismSimConfig config,
			SimTalonFXIO masterIO,
			SimTalonFXIO[] followerIOs) {
		this.sim = sim;
		this.masterIO = masterIO;
		this.followerIOs = followerIOs;
		this.frictionVoltage = config.frictionVoltage;
		this.timestampDeltaCalculator = new TimestampDeltaCalculator();
		SimulatedBattery.addElectricalAppliances(this::getTotalCurrentDraw);
		SimulatedArena.getInstance().addCustomSimulation(this);
	}

	/**
	 * Applies a simple Coulomb-like friction deadband to the motor terminal
	 * voltage.
	 * If the absolute voltage is below {@code frictionVoltage}, returns zero.
	 * Otherwise, subtracts
	 * (or adds) the friction voltage in the sign direction.
	 *
	 *
	 * @param motorVoltage    The commanded motor terminal voltage (V).
	 * @param frictionVoltage The effective friction drop (V) to model static
	 *                        friction/Coulomb losses.
	 * @return Adjusted motor voltage (V) after friction.
	 */
	protected double addFriction(double motorVoltage, double frictionVoltage) {
		if (Math.abs(motorVoltage) < frictionVoltage) {
			motorVoltage = 0.0;
		} else if (motorVoltage > 0.0) {
			motorVoltage -= frictionVoltage;
		} else {
			motorVoltage += frictionVoltage;
		}
		return motorVoltage;
	}

	/**
	 * Core simulation loop. Called periodically by the {@link SimulatedArena}.
	 *
	 * <ol>
	 * <li>Push current battery voltage to Talon sim via {@code setSupplyVoltage} so
	 * Phoenix can scale duty→volts.</li>
	 * <li>Read Phoenix’s commanded voltage via {@code getMotorVoltage()}.</li>
	 * <li>Optionally invert/sign-adjust and apply to the plant input.</li>
	 * <li>Integrate plant by {@code dt} seconds.</li>
	 * <li>Compute rotor position/velocity and write them to the Talon sim's
	 * sensor.</li>
	 * <li>Mirror rotor signals to all follower Talons.</li>
	 * </ol>
	 *
	 * <p>
	 * <b>Threading:</b> Avoid heavy work here; keep it deterministic and fast.
	 */
	@Override
	public void simulationSubTick(int subTick) {
		var simState = this.masterIO.getSimState();

		double busVolts = SimulatedBattery.getBatteryVoltage().in(Units.Volts);
		simState.setSupplyVoltage(busVolts);

		double simVoltage = this.addFriction(simState.getMotorVoltage(), this.frictionVoltage);

		this.sim.setInput(simVoltage);
		Logger.recordOutput(this.key + "/SimulatorVoltage", simVoltage);

		double dt = this.timestampDeltaCalculator.update();
		sim.update(dt);

		// If a manual rotor position override is present, apply it to the simulated
		// rotor position.
		// This allows for external control of the rotor position during simulation.
		this.manualRotorPositionOverride.getAndSet(Optional.empty())
			.ifPresent(pos -> this.setSimulatedRotorPositionRotations(pos));
		this.manualRotorVelocityOverride.getAndSet(Optional.empty())
			.ifPresent(vel -> this.setSimulatedRotorVelocityRPS(vel));

		double rotorPosition = this.getSimulatedRotorPositionRotations();
		Logger.recordOutput(this.key + "/SimulatorRotorPosition", rotorPosition);

		double rotorVel = this.getSimulatedRotorVelocityRPS();
		Logger.recordOutput(this.key + "/SimulatorRotorVel", rotorVel);

		double rotorAcceleration = (rotorVel - this.lastRotorVelocityRPS) / dt;
		Logger.recordOutput(this.key + "/SimulatorAcceleration", rotorAcceleration);

		this.lastRotorVelocityRPS = rotorVel;

		simState.setRawRotorPosition(rotorPosition);
		simState.setRotorVelocity(rotorVel);
		simState.setRotorAcceleration(rotorAcceleration);

		for (SimTalonFXIO simTalonFXIO : this.followerIOs) {
			var st = simTalonFXIO.getSimState();
			st.setSupplyVoltage(busVolts);
			st.setRawRotorPosition(rotorPosition);
			st.setRotorVelocity(rotorVel);
			st.setRotorAcceleration(rotorAcceleration);
		}
	}

	private Current getTotalCurrentDraw() {
		Current totalCurrent = masterIO.getSimState().getSupplyCurrentMeasure();
		for (SimTalonFXIO followerIO : followerIOs) {
			totalCurrent = totalCurrent.plus(followerIO.getSimState().getSupplyCurrentMeasure());
		}
		return totalCurrent;
	}

	/**
	 * Returns the current <b>rotor position</b> in rotations (not mechanism
	 * rotations).
	 *
	 * @return The current <b>rotor position</b> in rotations (not mechanism
	 *         rotations).
	 */
	protected abstract double getSimulatedRotorPositionRotations();

	/**
	 * Returns the current <b>rotor velocity</b> in rotations-per-second (RPS).
	 *
	 * @return The current <b>rotor velocity</b> in rotations-per-second (RPS).
	 */
	protected abstract double getSimulatedRotorVelocityRPS();


	protected void setSimulatedRotorPositionRotations(double rotorPositionRotations) {
		throw new UnsupportedOperationException(
			"Manual rotor position override not supported for this mechanism sim"
		);
	};

	protected void setSimulatedRotorVelocityRPS(double rotorVelocityRPS) {
		throw new UnsupportedOperationException(
			"Manual rotor velocity override not supported for this mechanism sim"
		);
	}

	/**
	 * Sets the friction voltage for the mechanism simulation.
	 * This value represents the voltage that counteracts motion due to friction.
	 *
	 * @param voltage The friction voltage to set, in volts. cant be neg.
	 */
	public void setFrictionVoltage(double voltage) {
		this.frictionVoltage = Math.abs(voltage);
	}

	public void resetSimRotorPosition(double rotorPositionRotations) {
		manualRotorPositionOverride.set(Optional.of(rotorPositionRotations));
	}

	public void resetSimRotorVelocity(double rotorVelocityRPS) {
		manualRotorVelocityOverride.set(Optional.of(rotorVelocityRPS));
	}

}
