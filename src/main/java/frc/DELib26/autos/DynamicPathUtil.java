package frc.DELib26.autos;

import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.IdealStartingState;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.RotationTarget;
import com.pathplanner.lib.path.Waypoint;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.units.measure.LinearVelocity;

import java.util.ArrayList;
import java.util.List;

public class DynamicPathUtil {

	public static PathPlannerPath buildPathMatchRotationEveryPoint(
		List<Pose2d> poses,
		PathConstraints constraints,
		LinearVelocity startVelocity,
		LinearVelocity endVelocity
	) {
		if (poses.size() < 2) {
			throw new IllegalArgumentException("Need at least 2 poses");
		}

		// Keep the rotations you WANT
		List<RotationTarget> rotationTargets = new ArrayList<>();
		for (int i = 0; i < poses.size(); i++) {
			rotationTargets.add(new RotationTarget(i, poses.get(i).getRotation()));
		}

		// Use tangent rotations ONLY for curve generation
		List<Pose2d> tangentPoses = withTangentRotations(poses);
		List<Waypoint> waypoints = PathPlannerPath.waypointsFromPoses(tangentPoses);

		PathPlannerPath path = new PathPlannerPath(
			waypoints,
			rotationTargets,
			List.of(), // pointTowardsZones
			List.of(), // constraintZones
			List.of(), // eventMarkers
			constraints,
			new IdealStartingState(startVelocity, poses.get(0).getRotation()),
			new GoalEndState(endVelocity, poses.get(poses.size() - 1).getRotation()),
			false
		);

		path.preventFlipping = true;
		return path;
	}

	private static List<Pose2d> withTangentRotations(List<Pose2d> poses) {
		int n = poses.size();
		if (n < 2) return poses;

		List<Pose2d> out = new ArrayList<>(n);
		for (int i = 0; i < n; i++) {
			Rotation2d rot;
			if (i == 0) {
				rot = poses.get(1).getTranslation().minus(poses.get(0).getTranslation()).getAngle();
			} else if (i == n - 1) {
				rot = poses.get(n - 1).getTranslation().minus(poses.get(n - 2).getTranslation()).getAngle();
			} else {
				Translation2d a = poses.get(i).getTranslation().minus(poses.get(i - 1).getTranslation());
				Translation2d b = poses.get(i + 1).getTranslation().minus(poses.get(i).getTranslation());
				Translation2d sum = new Translation2d(a.getX() + b.getX(), a.getY() + b.getY());
				rot = (sum.getNorm() < 1e-9) ? b.getAngle() : sum.getAngle();
			}
			out.add(new Pose2d(poses.get(i).getTranslation(), rot));
		}
		return out;
	}
}
