package frc.DELib26.util;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;

import java.util.List;

public class FieldUtil {

	/**
	 * Gets the current alliance, or defaults to Blue if not yet known.
	 *
	 * @return The current alliance, or Blue if unknown.
	 */
	public static DriverStation.Alliance getAllianceOrDefault() {
		return DriverStation.getAlliance().orElse(DriverStation.Alliance.Blue);
	}

	/**
	 * Checks whether the robot is on the blue alliance.
	 * If the alliance is not yet known, it defaults to blue.
	 *
	 * @return true if alliance is Blue OR alliance is unknown (defaulted to Blue).
	 */
	public static boolean isBlueAllianceOrDefault() {
		return DriverStation.getAlliance().orElse(DriverStation.Alliance.Blue) == DriverStation.Alliance.Blue;
	}

	/**
	 * Returns true only if the alliance is known AND it is Blue.
	 * Unlike {@link #isBlueAllianceOrDefault()}, this method will return false
	 * if the alliance is not yet reported.
	 *
	 * @return true if explicitly on Blue alliance.
	 */
	public static boolean isBlueAlliance() {
		return DriverStation.getAlliance().isPresent() && isBlueAllianceOrDefault();
	}

	/**
	 * Returns true only if the alliance is known AND it is Red.
	 * If the alliance is unknown, this will return false.
	 *
	 * @return true if explicitly on Red alliance.
	 */
	public static boolean isRedAlliance() {
		return DriverStation.getAlliance().isPresent() && !isBlueAllianceOrDefault();
	}

	/**
	 * Gets the forward heading of the robot relative to the field,
	 * based on alliance color.
	 *
	 * - Blue alliance forward = 0 degrees (positive X direction).
	 * - Red alliance forward = 180 degrees (negative X direction).
	 *
	 * @return Rotation2d representing the forward direction for the current alliance.
	 */
	public static Rotation2d getAllianceForwardHeading() {
		return isBlueAllianceOrDefault() ? Rotation2d.kZero : Rotation2d.k180deg;
	}

	/**
	 * Returns +1 if the alliance’s forward is +X (Blue),
	 * or -1 if the alliance’s forward is -X (Red).
	 * Useful for flipping trajectories or joystick controls.
	 *
	 * @return 1 for Blue, -1 for Red.
	 */
	public static int getAllianceDirectionMultiplier() {
		return isBlueAllianceOrDefault() ? 1 : -1;
	}

	public static int getAllianceLocation() {
		return DriverStation.getLocation().orElse(1);
	}

	/**
	 * Converts a Pose2d from Blue alliance coordinates to Red alliance coordinates.
	 * @param pose The original Pose2d to mirror.
	 * @param fieldLengthMeters The length of the field in meters.
	 * @return A new Pose2d mirrored across the field length.
	 */
	public static Pose2d mirrorPoseAcrossField(Pose2d pose, double fieldLengthMeters) {
		return new Pose2d(
				fieldLengthMeters - pose.getX(), pose.getY(), pose.getRotation().plus(Rotation2d.k180deg));
	}

	/**
	 * Mirrors a Pose2d across both the X and Y axes of the field.
	 * Useful for converting positions between alliances.
	 * @param pose The original Pose2d to mirror.
	 * @param fieldLength The length of the field in meters.
	 * @param fieldWidth The width of the field in meters.
	 * @return A new Pose2d mirrored across both axes.
	 */
	public static Pose2d mirrorBothAxes(Pose2d pose, double fieldLength, double fieldWidth) {
		return new Pose2d(
				mirrorBothAxes(pose.getTranslation(), fieldLength, fieldWidth),
				mirrorBothAxes(pose.getRotation(), fieldLength, fieldWidth)
			);
	}

	/**
	 * Mirrors a Translation2d across both the X and Y axes of the field.
	 * Useful for converting positions between alliances.
	 * @param translation The original Translation2d to mirror.
	 * @param fieldLength The length of the field in meters.
	 * @param fieldWidth The width of the field in meters.
	 * @return A new Translation2d mirrored across both axes.
	 */
	public static Translation2d mirrorBothAxes(Translation2d translation, double fieldLength, double fieldWidth) {
		return new Translation2d(
				fieldLength - translation.getX(),
				fieldWidth - translation.getY());
	}

	/**
	 * Mirrors a Rotation2d across both the X and Y axes of the field.
	 * Useful for converting positions between alliances.
	 * @param rotation The original Rotation2d to mirror.
	 * @param fieldLength The length of the field in meters.
	 * @param fieldWidth The width of the field in meters.
	 * @return A new Rotation2d mirrored across both axes.
	 */
	public static Rotation2d mirrorBothAxes(Rotation2d rotation, double fieldLength, double fieldWidth) {
		return rotation.plus(Rotation2d.k180deg);
	}

	/**
	 * Changes the coordinate system of a given point.
	 * @param point the point in the original coordinate system
	 * @param center the center of the new coordinate system, placed in the old, the rotation2d pointing to the x axis direction
	 * @param invertX if false, center = 0,0,0 will point right and up. if true center = 0,0,0 will point left and up
	 * @return the point in the new coordinate system
	 */
	public static Pose2d changeCoordinateSystem(Pose2d point, Pose2d center, boolean invertX) {
		Translation2d temp = center.getTranslation().plus(point.getTranslation().minus(center.getTranslation()).rotateBy(center.getRotation())); // rotates point around center by center angle
		if(invertX) {
			temp = new Translation2d(-temp.getX(), temp.getY());
		}
		return new Pose2d(temp.minus(center.getTranslation()),
		point.getRotation().plus(center.getRotation()));
	}

	/**
	 * its O(n) but n is small plus the only way to optimize is to use a spatial data structure and i dont know how to do that
	 * and having preset segregated lists for each tag will take more memory
	 * @param current the current pose
	 * @param candidates the list of candidates to check
	 * @return the nearest pose by position
	 */
	public static Pose2d nearestByPosition(Pose2d current, List<Pose2d> candidates) {
		Pose2d best = null;
		double bestDist2 = Double.POSITIVE_INFINITY;

		final double cx = current.getX();
		final double cy = current.getY();

		for (Pose2d p : candidates) {
			double dx = p.getX() - cx;
			double dy = p.getY() - cy;
			// We dont use sqrt because its monotonic and squareroot is expensive, plus we dont care about the actual distance
			double d2 = Math.pow(dx, 2) + Math.pow(dy, 2);
			if (d2 < bestDist2) {
				bestDist2 = d2;
				best = p;
			}
		}
		return best;
	}

	/**
	 * Checks if two poses are within a specified distance and angle tolerance.
	 * @param pose1 The first pose.
	 * @param pose2 The second pose.
	 * @param distanceTolerance The maximum allowed distance between the poses.
	 * @param angleTolerance The maximum allowed angle difference between the poses in degrees.
	 * @return true if the poses are within the specified tolerances.
	 */
	public static boolean arePosesClose(Pose2d pose1, Pose2d pose2, double distanceTolerance, double angleTolerance) {
		double distance = pose1.getTranslation().getDistance(pose2.getTranslation());
		double angleDifference = getAngleDifferenceDegrees(pose1, pose2);
		return distance < distanceTolerance && angleDifference < angleTolerance;
	}

	/**
	 * Calculates the absolute angle difference in degrees between two poses.
	 * Results are in the range [0, 180] degrees.
	 * @param pose1 The first pose.
	 * @param pose2 The second pose.
	 * @return The angle difference in degrees between the two poses.
	 * @see #getAngleDifferenceDegrees(Rotation2d, Rotation2d)
	 */
	public static double getAngleDifferenceDegrees(Pose2d pose1, Pose2d pose2) {
		return getAngleDifferenceDegrees(pose1.getRotation(), pose2.getRotation());
	}

	/**
	 * Calculates the absolute angle difference in degrees between two Rotation2d angles.
	 * Results are in the range [0, 180] degrees.
	 *
	 * @param angle1 The first Rotation2d angle.
	 * @param angle2 The second Rotation2d angle.
	 * @return The absolute angle difference in degrees.
	 */
	public static double getAngleDifferenceDegrees(Rotation2d angle1, Rotation2d angle2) {
		return Math.abs(getAngleErrorDegrees(angle1, angle2));
	}

	/**
	 * Calculates the angle error in degrees between two Rotation2d angles.
	 * Results are in the range [-180, 180) degree
	 *
	 * @param angle1 The first Rotation2d angle.
	 * @param angle2 The second Rotation2d angle.
	 * @return The angle error in degrees.
	 */
	public static double getAngleErrorDegrees(Rotation2d angle1, Rotation2d angle2) {
		return MathUtil.inputModulus(angle1.minus(angle2).getDegrees(), -180.0, 180.0);
	}


	/**
	 * Converts a 3D transform to a 2D transform by discarding the Z components.
	 * The resulting Transform2d contains the X and Y translation from the Transform3d,
	 * and the rotation around the Z axis (yaw).
	 *
	 * @param t3 The Transform3d to convert.
	 * @return A Transform2d representing the 2D equivalent of the input Transform3d.
	 */
	public static Transform2d to2d(Transform3d t3) {
		return new Transform2d(
			t3.getTranslation().toTranslation2d(),
			Rotation2d.fromRadians(t3.getRotation().getZ())
		);
	}

}
