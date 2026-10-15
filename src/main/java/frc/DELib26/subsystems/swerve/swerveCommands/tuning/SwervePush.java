package frc.DELib26.subsystems.swerve.swerveCommands.tuning;

import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.swerve.SwerveRequest;
import edu.wpi.first.wpilibj2.command.Command;
import frc.DELib26.subsystems.swerve.SwerveSubsystem;

import java.util.Objects;

/**
 * Holds the robot in a passive "push" state (nutral) until the exit condition becomes true.
 * - Sets all swerve modules to coast and issues a SwerveRequest.Idle() continuously.
 * - Restores the original wanted drive state on end.
 */
public class SwervePush extends Command {
	private final SwerveSubsystem swerve;

	public SwervePush(SwerveSubsystem swerve) {
		this.swerve = Objects.requireNonNull(swerve);
		addRequirements(swerve);
		setName("SwervePush");
	}

	@Override
	public void initialize() {
		this.swerve.forEachSteerMotor(m -> m.setNeutralMode(NeutralModeValue.Coast));
	}

	@Override
	public void execute() {
		this.swerve.setSwerveState(new SwerveRequest.Idle());
	}

	@Override
	public void end(boolean interrupted) {
		this.swerve.forEachSteerMotor(m -> m.setNeutralMode(NeutralModeValue.Brake));
	}
}
