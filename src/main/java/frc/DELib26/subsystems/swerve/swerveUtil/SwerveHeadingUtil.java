package frc.DELib26.subsystems.swerve.swerveUtil;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import frc.DELib26.subsystems.swerve.SwerveSubsystem;

import java.util.function.Supplier;

public class SwerveHeadingUtil {
	private SwerveHeadingUtil() {
	}

	/**
	 * Converts a field Translation2d target into a desired robot heading supplier.
	 */
	public static Rotation2d computeFieldPointHeading(
			SwerveSubsystem swerve,
			Supplier<Translation2d> targetSupplier,
			Transform2d robotToComponent) {
		Pose2d robotPose = swerve.getPose();
		Translation2d target = targetSupplier.get();

		Translation2d componentFieldPos = robotPose.transformBy(
				new Transform2d(robotToComponent.getTranslation(), Rotation2d.kZero)).getTranslation();

		Translation2d delta = target.minus(componentFieldPos);
		if (delta.getNorm() < SwerveSubsystemConstants.HEADING_STABILITY_DISTANCE_METERS) {
			return robotPose.getRotation();
		}

		Rotation2d toTarget = delta.getAngle();
		return toTarget.minus(robotToComponent.getRotation());
	}
}
