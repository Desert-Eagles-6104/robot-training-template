package frc.DELib26.subsystems.swerve.swerveCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.DELib26.subsystems.swerve.SwerveSubsystem;

public class TeleopDrive extends Command {
	private final SwerveSubsystem swerve;

	public TeleopDrive(SwerveSubsystem swerve) {
		this.swerve = swerve;
		this.addRequirements(swerve);
	}

	@Override
	public void execute() {
		this.swerve.teleopDrive();
	}
}
