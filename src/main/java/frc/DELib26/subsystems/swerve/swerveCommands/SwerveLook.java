package frc.DELib26.subsystems.swerve.swerveCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.DELib26.subsystems.swerve.SwerveSubsystem;
public class SwerveLook extends Command {

	private final SwerveSubsystem swerve;

	public SwerveLook(SwerveSubsystem swerve) {
		this.swerve = swerve;
		this.addRequirements(swerve);
	}

	@Override
	public void execute() {
		this.swerve.xLock();
	}

}
