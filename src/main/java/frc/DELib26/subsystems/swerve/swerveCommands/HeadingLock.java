package frc.DELib26.subsystems.swerve.swerveCommands;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.DELib26.subsystems.swerve.SwerveSubsystem;
import frc.DELib26.subsystems.swerve.swerveUtil.SwerveHeadingUtil;
import frc.DELib26.subsystems.swerve.swerveUtil.SwerveSubsystemConstants;
import frc.DELib26.util.TimestampDeltaCalculator;

import java.util.function.Supplier;

/**
 * While scheduled:
 * - Driver controls X/Y translation normally (teleopDrive)
 * - Robot continuously rotates to face a field Translation2d target
 */
public class HeadingLock extends Command {
	protected final SwerveSubsystem swerve;
	protected final Supplier<Rotation2d> desiredHeadingSupplier;
	protected final Supplier<Rotation2d> toleranceSupplier;
	protected Rotation2d lastDesiredHeading;
	protected TimestampDeltaCalculator timestampDeltaCalculator = new TimestampDeltaCalculator();

	public HeadingLock(
		SwerveSubsystem swerve,
		Supplier<Rotation2d> desiredHeadingSupplier,
		Supplier<Rotation2d> toleranceSupplier
	) {
		this.swerve = swerve;
		this.desiredHeadingSupplier = desiredHeadingSupplier;
		this.toleranceSupplier = toleranceSupplier;

		addRequirements(swerve);
		setName("HeadingLock");
	}

	public HeadingLock(
		SwerveSubsystem swerve,
		Supplier<Translation2d> targetSupplier,
		Transform2d robotToComponent
	) {
		this(
			swerve,
			() -> SwerveHeadingUtil.computeFieldPointHeading(swerve, targetSupplier, robotToComponent),
			() -> SwerveSubsystemConstants.ROTATION_ERROR_MARGIN_FOR_ROTATION_LOCK
		);
	}

	@Override
	public void initialize() {
		swerve.resetControllers();
		// Start from "where we are now" so we don't snap to 0 degs
		this.lastDesiredHeading = swerve.getPose().getRotation();
		this.timestampDeltaCalculator.reset();
	}

	@Override
	public void execute() {
		swerve.getHeadingController().setTolerance(toleranceSupplier.get().getRadians());
		swerve.setDesiredHeadingForHeadingLock(this.desiredHeadingSupplier.get());
	}

	@Override
	public void end(boolean interrupted) {
		swerve.resetControllers();
	}

	@Override
	public boolean isFinished() {
		return false; // intended for "whileHeld"/"whileTrue"
	}
}