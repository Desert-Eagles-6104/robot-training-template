package frc.DELib26.subsystems.swerve.swerveCommands;

import edu.wpi.first.math.geometry.Pose2d;
import frc.DELib26.subsystems.swerve.SwerveSubsystem;

import java.util.function.Supplier;

public class DriveToPoint extends DriveToChangingPoint {
	private Pose2d lockedTarget;

	public DriveToPoint(SwerveSubsystem swerve, Supplier<Pose2d> targetSupplier) {
		super(swerve, targetSupplier);
		setName("DriveToPoint");
	}

	@Override
	public void initialize() {
		super.initialize();
		this.lockedTarget = this.targetSupplier.get(); // Lock in the target
	}

	@Override
	protected Pose2d getTarget() {
		return this.lockedTarget;
	}

}
