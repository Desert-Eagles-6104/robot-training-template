package frc.DELib26.subsystems.swerve.swerveCommands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.DELib26.subsystems.swerve.SwerveSubsystem;

import java.util.function.Supplier;

public class DriveToChangingPoint extends Command {
	protected final SwerveSubsystem swerve;
	protected Supplier<Pose2d> targetSupplier;

	public DriveToChangingPoint(SwerveSubsystem swerve, Supplier<Pose2d> targetSupplier) {
		this.swerve = swerve;
		this.targetSupplier = targetSupplier;
		this.addRequirements(swerve);
		this.setName("DriveToChangingPoint");
	}

	@Override
	public void initialize() {
		this.swerve.resetControllers();
	}

	protected Pose2d getTarget() {
		return this.targetSupplier.get();
	}

	@Override
	public void execute() {
		this.swerve.setDesiredPoseForDriveToPoint(this.getTarget());
	}

	@Override
	public void end(boolean interrupted) {
		if (!interrupted) {
			this.swerve.stop();
		}
	}

	@Override
	public boolean isFinished() {
		// to make it not jitter at the goal and a isDriveToPointAtSetpoint flag to stop
		// it from over correcting at the end (or just fix the PID/underling logic) its
		// not perfect yet though either way and needs more testing and debugging
		// Especially on repeated commands With different set points and with the
		// heading being different.Then the original

		return (this.swerve.isAtDriveToPointSetpoint() && this.swerve.isAtDriveToPointDesiredRotation())
				|| this.swerve.isManualDriveEnabled();
	}
}
