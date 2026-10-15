package frc.DELib26.subsystems.swerve.swerveCommands;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj2.command.Command;
import frc.DELib26.subsystems.swerve.SwerveSubsystem;
import frc.DELib26.subsystems.swerve.swerveUtil.SwerveHeadingUtil;
import frc.DELib26.subsystems.swerve.swerveUtil.SwerveSubsystemConstants;

import java.util.function.Supplier;

public class FaceHeading extends Command {
	private final SwerveSubsystem swerve;
	private final Supplier<Rotation2d> desiredHeadingSupplier;

	private final Supplier<Rotation2d> toleranceSupplier;
	private final int stableCyclesRequired;

	private int stableCycles = 0;
	private final double vEps;
	private final double omegaEpsRadPerSec;

	public FaceHeading(
			SwerveSubsystem swerve,
			Supplier<Rotation2d> desiredHeadingSupplier) {
		this.swerve = swerve;
		this.desiredHeadingSupplier = desiredHeadingSupplier;

		this.stableCyclesRequired = 1;
		this.vEps = -1.0;
		this.omegaEpsRadPerSec = -1.0;
		this.toleranceSupplier = () -> SwerveSubsystemConstants.ROTATION_ERROR_MARGIN_FOR_ROTATION_LOCK;

		addRequirements(swerve);
		setName("FaceFieldPoint");
	}

	public FaceHeading(
			SwerveSubsystem swerve,
			Supplier<Translation2d> targetSupplier,
			Transform2d robotOffset,
			Supplier<Rotation2d> toleranceSupplier,
			int stableCyclesRequired,
			double vEps,
			AngularVelocity omegaEps) {
		this.swerve = swerve;
		this.desiredHeadingSupplier = () -> SwerveHeadingUtil.computeFieldPointHeading(swerve, targetSupplier,
				robotOffset);

		// override defaults with provided params
		this.stableCyclesRequired = Math.max(1, stableCyclesRequired);
		this.vEps = vEps;
		this.omegaEpsRadPerSec = omegaEps.in(Units.RadiansPerSecond);
		this.toleranceSupplier = toleranceSupplier;
		addRequirements(swerve);
		setName("FaceFieldPoint");
	}

	@Override
	public void initialize() {
		swerve.resetControllers();

		if (omegaEpsRadPerSec >= 0) {
			swerve.getHeadingController().setTolerance(
					toleranceSupplier.get().getRadians(),
					omegaEpsRadPerSec);
		} else {
			swerve.getHeadingController().setTolerance(
					SwerveSubsystemConstants.ROTATION_ERROR_MARGIN_FOR_ROTATION_LOCK.getRadians(),
					SwerveSubsystemConstants.ROTATION_LOCK_MIN_ANGULAR_VELOCITY_FOR_ZERO_VALUE_RAD_PER_SEC);
		}

		this.stableCycles = 0;

		// If we're already at the setpoint and stopped, end immediately.
		if (isRobotStoppedAtHeading()) {
			this.end(false);
		}
	}

	@Override
	public void execute() {
		swerve.faceHeading(desiredHeadingSupplier.get());
	}

	@Override
	public void end(boolean interrupted) {
		swerve.resetControllers();
		swerve.stop();
	}

	@Override
	public boolean isFinished() {
		if (isRobotStoppedAtHeading()) {
			stableCycles++;
		} else {
			stableCycles = 0;
		}

		return stableCycles >= stableCyclesRequired;
	}

	protected boolean isRobotStoppedAtHeading() {
		ChassisSpeeds speeds = swerve.getSwerveInputs().RobotSpeeds;
		double linearSpeed = Math.hypot(speeds.vxMetersPerSecond, speeds.vyMetersPerSecond);

		boolean belowLinearThreshold = (vEps < 0) || (linearSpeed < vEps);
		boolean belowAngularThreshold = (omegaEpsRadPerSec < 0)
				|| (Math.abs(speeds.omegaRadiansPerSecond) < omegaEpsRadPerSec);

		boolean isStopped = belowLinearThreshold && belowAngularThreshold;
		boolean atHeading = swerve.isAtHeadingLockRotationSetpoint();

		return atHeading && isStopped;
	}
}