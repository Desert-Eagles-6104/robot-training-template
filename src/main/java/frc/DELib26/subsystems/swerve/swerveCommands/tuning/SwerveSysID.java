package frc.DELib26.subsystems.swerve.swerveCommands.tuning;

import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.swerve.SwerveRequest;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.DELib26.subsystems.swerve.SwerveSubsystem;
import frc.DELib26.subsystems.swerve.swerveUtil.SwerveConstants;
import frc.DELib26.subsystems.swerve.swerveUtil.SysIdMechanism;
import frc.DELib26.util.Conversions;

import java.util.ArrayList;
import java.util.List;
import java.util.function.Consumer;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.DegreesPerSecond;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Volts;

/**
 * SysId command for characterizing the swerve drivetrain.
 * <p>
 * You choose which mechanism to run (translation / rotation / steer) and it
 * will run dynamic + quasistatic tests forward and reverse.
 */
public class SwerveSysID extends SequentialCommandGroup {
	private final String[] names = { "FL", "FR", "BL", "BR" };
	private final SwerveSubsystem swerveSubsystem;
	private final SwerveConstants swerveConstants;
	/** Last voltage commanded by SysId (used for logging). */
	private Voltage lastSysIdVolts = Volts.of(0);

	/**
	 * Build a SysId sequence for the given mechanism.
	 *
	 * @param swerve Swerve subsystem
	 * @param mechanism Which mechanism to characterize
	 * @param waitBetweenRuns If true, waits 1 second between each test
	 */
	public SwerveSysID(SwerveSubsystem swerve, SwerveConstants swerveConstants, SysIdMechanism mechanism, boolean waitBetweenRuns) {
		this.swerveSubsystem = swerve;
		this.swerveConstants = swerveConstants;

		// Build the routine for the requested mechanism
		SysIdRoutine routine = createRoutine(mechanism);

		// Build the step sequence: dynamic fwd/rev + quasistatic fwd/rev
		List<Command> steps = new ArrayList<>();

		steps.add(routine.dynamic(Direction.kForward));
		if (waitBetweenRuns) steps.add(new WaitCommand(1));

		steps.add(routine.dynamic(Direction.kReverse));
		if (waitBetweenRuns) steps.add(new WaitCommand(1));

		steps.add(routine.quasistatic(Direction.kForward));
		if (waitBetweenRuns) steps.add(new WaitCommand(1));

		steps.add(routine.quasistatic(Direction.kReverse));

		Command sequence = Commands.sequence(steps.toArray(Command[]::new))
				.andThen(new PrintCommand("SysId complete"));

		addRequirements(swerve);
		addCommands(sequence);
	}

	/**
	 * Create the SysIdRoutine for the selected mechanism.
	 */
	private SysIdRoutine createRoutine(SysIdMechanism mechanism) {
		return switch (mechanism) {
			case SWERVE_TRANSLATION -> this.createTranslationRoutine();
			case SWERVE_ROTATION -> this.createRotationRoutine();
			case SWERVE_STEER -> this.createSteerRoutine();
		};
	}

	/**
	 * SysId routine for drive (translation) characterization.
	 * Used to tune drive velocity PID + FF.
	 */
	private SysIdRoutine createTranslationRoutine() {
		return new SysIdRoutine(
				this.swerveConstants.TRANSLATION_SYS_ID_CONFIG,
				new SysIdRoutine.Mechanism(
						this.getConsumer(SysIdMechanism.SWERVE_TRANSLATION),
						log -> {
							var swerveInputs = swerveSubsystem.getSwerveInputs();

							double sumPos = 0.0;
							double sumVel = 0.0;

							for (int i = 0; i < swerveInputs.ModulePositions.length; i++) {
								var modulePos = swerveInputs.ModulePositions[i];
								var moduleState = swerveInputs.ModuleStates[i];

								sumPos += modulePos.distanceMeters;
								sumVel += moduleState.speedMetersPerSecond;

								log.motor(names[i])
										.voltage(lastSysIdVolts)
										.linearPosition(Meters.of(modulePos.distanceMeters))
										.linearVelocity(MetersPerSecond.of(moduleState.speedMetersPerSecond));
							}

							double avgPos = sumPos / swerveInputs.ModulePositions.length;
							double avgVel = sumVel / swerveInputs.ModuleStates.length;

							log.motor("Drive")
									.voltage(lastSysIdVolts)
									.linearPosition(Meters.of(avgPos))
									.linearVelocity(MetersPerSecond.of(avgVel));
						},
						swerveSubsystem
				)
		);
	}

	/**
	 * SysId routine for robot rotation (heading) characterization.
	 * Used to tune your heading controller PID + FF.
	 */
	private SysIdRoutine createRotationRoutine() {
		return new SysIdRoutine(
				this.swerveConstants.ROTATION_SYS_ID_CONFIG,
				new SysIdRoutine.Mechanism(
						getConsumer(SysIdMechanism.SWERVE_ROTATION),
						log -> {
							var inputs = swerveSubsystem.getPigeon2Inputs();

							log.motor("Yaw")
									.voltage(lastSysIdVolts)
									.angularPosition(Degrees.of(inputs.yawDegs))
									.angularVelocity(DegreesPerSecond.of(inputs.angularYawVelocityDPS));
						},
						swerveSubsystem
				)
		);
	}

	/**
	 * SysId routine for module steer characterization.
	 * Used to tune steer response (mostly kS / kV, and sanity-check friction + ratios).
	 */
	private SysIdRoutine createSteerRoutine() {
		final double[] prevDeg = new double[4];
		final double[] prevTime = new double[4];
		final boolean[] hasPrev = new boolean[4];

		return new SysIdRoutine(
				this.swerveConstants.STEER_SYS_ID_CONFIG,
				new SysIdRoutine.Mechanism(
						getConsumer(SysIdMechanism.SWERVE_STEER),
						log -> {
							var swerveInputs = swerveSubsystem.getSwerveInputs();

							double now = Utils.getCurrentTimeSeconds();

							double sumAngleDeg = 0.0;
							double sumRateDps = 0.0;
							int n = swerveInputs.ModuleStates.length;

							for (int i = 0; i < n; i++) {
								var moduleState = swerveInputs.ModuleStates[i];

								double angleDeg = moduleState.angle.getDegrees();

								double rateDps = 0.0;
								if (hasPrev[i]) {
									double dt = now - prevTime[i];
									if (dt > 1e-4) {
										double dDeg = Conversions.wrapTo180Degrees(angleDeg - prevDeg[i]);
										rateDps = dDeg / dt;
									}
								} else {
									hasPrev[i] = true;
								}

								prevDeg[i] = angleDeg;
								prevTime[i] = now;

								sumAngleDeg += angleDeg;
								sumRateDps += rateDps;

								log.motor(names[i] + "_Steer")
										.voltage(lastSysIdVolts)
										.angularPosition(Degrees.of(angleDeg))
										.angularVelocity(DegreesPerSecond.of(rateDps));
							}

							log.motor("Steer")
									.voltage(lastSysIdVolts)
									.angularPosition(Degrees.of(sumAngleDeg / n))
									.angularVelocity(DegreesPerSecond.of(sumRateDps / n));
						},
						swerveSubsystem
				)
		);
	}

	/**
	 * Consumer used by SysId to apply the requested voltage to the selected mechanism.
	 * Also updates {@link #lastSysIdVolts} for logging.
	 */
	private Consumer<Voltage> getConsumer(SysIdMechanism mechanism) {
		return switch (mechanism) {
			case SWERVE_TRANSLATION -> output -> {
				lastSysIdVolts = output;
				swerveSubsystem.setSwerveState(
						new SwerveRequest.SysIdSwerveTranslation().withVolts(output));
			};
			case SWERVE_ROTATION -> output -> {
				lastSysIdVolts = output;
				swerveSubsystem.setSwerveState(
						new SwerveRequest.SysIdSwerveRotation().withRotationalRate(output.in(Volts)));
			};
			case SWERVE_STEER -> output -> {
				lastSysIdVolts = output;
				swerveSubsystem.setSwerveState(
						new SwerveRequest.SysIdSwerveSteerGains().withVolts(output));
			};
		};
	}
}
