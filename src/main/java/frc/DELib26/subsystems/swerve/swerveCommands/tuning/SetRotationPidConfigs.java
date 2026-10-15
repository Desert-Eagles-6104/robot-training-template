package frc.DELib26.subsystems.swerve.swerveCommands.tuning;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.DELib26.subsystems.swerve.SwerveSubsystem;

import java.util.function.DoubleSupplier;

public class SetRotationPidConfigs extends InstantCommand {
	public SetRotationPidConfigs(SwerveSubsystem swerve, DoubleSupplier p, DoubleSupplier i, DoubleSupplier d) {
		super(
			() -> swerve.getHeadingController().setPID(p.getAsDouble(), i.getAsDouble(), d.getAsDouble())
		);
	}
}
