package frc.DELib26.subsystems.motorSubsystems.commands;

import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.sysid.SysIdRoutineLog;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.DELib26.subsystems.motorSubsystems.motorBase.MotorSubsystemTalon;

import java.util.ArrayList;
import java.util.List;

/**
 * This is for the slot PID configs - they are run with mechanism rotation
 * units, so we also use rotations here.
 * It's important to make sure that the units in the SysId panel are in
 * rotations.
 */
public class MotorSubsystemSysId extends SequentialCommandGroup {
	private final MotorSubsystemTalon subsystem;
	private final SysIdRoutine routine;
	private Voltage lastSysIdVolts = Units.Volts.of(0);

	public MotorSubsystemSysId(SysIdRoutine.Config config, MotorSubsystemTalon subsystem, boolean wait) {
		this.subsystem = subsystem;
		this.routine = new SysIdRoutine(
				config,
				new SysIdRoutine.Mechanism(
						(volts) -> {
							this.lastSysIdVolts = volts;
							subsystem.runCharacterization(volts);
						},
						this::getLogger,
						subsystem));
		this.buildFullCharacterization(wait);
	}

	private void buildFullCharacterization(boolean wait) {
		List<Command> steps = new ArrayList<>();
		steps.add(new InstantCommand(() -> {
			this.subsystem.changeNeutralMode(NeutralModeValue.Coast);
			this.subsystem.controlForwardSoftLimit(false);
			this.subsystem.controlReverseSoftLimit(false);
			SignalLogger.start();
		}));

		steps.add(routine.quasistatic(Direction.kForward));
		if (wait)
			steps.add(new WaitCommand(0.5));
		steps.add(routine.quasistatic(Direction.kReverse));
		if (wait)
			steps.add(new WaitCommand(0.5));
		steps.add(routine.dynamic(Direction.kForward));
		if (wait)
			steps.add(new WaitCommand(0.5));
		steps.add(routine.dynamic(Direction.kReverse));

		steps.add(new InstantCommand(() -> {
			TalonFXConfiguration config = subsystem.configuration.master.getTalonFXConfig();
			this.subsystem.changeNeutralMode(config.MotorOutput.NeutralMode);
			this.subsystem.controlForwardSoftLimit(config.SoftwareLimitSwitch.ForwardSoftLimitEnable);
			this.subsystem.controlReverseSoftLimit(config.SoftwareLimitSwitch.ReverseSoftLimitEnable);
			SignalLogger.stop();
		}));

		Command seq = Commands.sequence(steps.toArray(new Command[0]))
				.andThen(new PrintCommand("Phoenix SysId Complete"));
		this.addCommands(seq);
	}

	private void getLogger(SysIdRoutineLog log) {
		log.motor(this.subsystem.getName() + "-master")
				.voltage(this.lastSysIdVolts)
				.angularPosition(this.subsystem.getMasterInputs().mechanismPosition)
				.angularVelocity(
						Units.RotationsPerSecond.of(this.subsystem.getMasterInputs().mechanismVelocityRotPerSec));
	}
}
