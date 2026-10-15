package frc.DELib26.util.geometry;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;

public class GeometryUtil {

	/**
	 * Checks if a point is inside or on the edge of a rectangle defined by two opposite corners.
	 *
	 * @param point The point to test
	 * @param a One corner of the rectangle
	 * @param b The opposite corner of the rectangle
	 * @return true if inside or on edge, false otherwise
	 * @throws IllegalArgumentException if the corners are the same point or form a line
	 */
	public static boolean isPointInsideRectangle(
		Translation2d point,
		Translation2d a,
		Translation2d b
	) {
		double minX = Math.min(a.getX(), b.getX());
		double maxX = Math.max(a.getX(), b.getX());
		double minY = Math.min(a.getY(), b.getY());
		double maxY = Math.max(a.getY(), b.getY());
		if (minX == maxX || minY == maxY) {
			throw new IllegalArgumentException("Rectangle corners cannot be the same point or form a line.");
		}
		return point.getX() >= minX && point.getX() <= maxX
			&& point.getY() >= minY && point.getY() <= maxY;
	}

	/**
	 * Determines if a point is inside a polygon using the ray-casting algorithm.
	 * @param point The point to test
	 * @param polygon Vertices of the polygon in order (CW or CCW). Can be closed or not.
	 * @return true if inside (or on the edge), false otherwise
	 */
	public static boolean isPointInsidePolygon(Translation2d point, Translation2d... polygon) {
		if (polygon == null || polygon.length < 3) return false;

		double x = point.getX();
		double y = point.getY();

		boolean inside = false;

		for (int i = 0, j = polygon.length - 1; i < polygon.length; j = i++) {
		double xi = polygon[i].getX(), yi = polygon[i].getY();
		double xj = polygon[j].getX(), yj = polygon[j].getY();

		// If point is exactly on the segment, treat as inside
		if (isPointOnSegment(x, y, xj, yj, xi, yi)) {
			return true;
		}

		// Ray casting: does the edge cross a horizontal ray to the right of point?
		boolean intersects = ((yi > y) != (yj > y))
			&& (x < (xj - xi) * (y - yi) / (yj - yi) + xi);

		if (intersects) inside = !inside;
		}

		return inside;
	}

	// Point-on-segment with a small tolerance
	private static boolean isPointOnSegment(
		double px, double py,
		double ax, double ay,
		double bx, double by
	) {
		double cross = (px - ax) * (by - ay) - (py - ay) * (bx - ax);
		double eps = 1e-9;
		if (Math.abs(cross) > eps) return false;

		double dot = (px - ax) * (bx - ax) + (py - ay) * (by - ay);
		if (dot < -eps) return false;

		double lenSq = (bx - ax) * (bx - ax) + (by - ay) * (by - ay);
		return dot <= lenSq + eps;
	}

	/**
	* Transforms a velocity along a translation.
	*
	* @param velocity The original velocity
	* @param transform The transform to the new position
	* @param currentRotation The current rotation of the robot
	* @return The new velocity
	*/
	public static ChassisSpeeds transformVelocity(
		ChassisSpeeds velocity,
		Translation2d transform,
		Rotation2d currentRotation
	) {
		double rx = transform.getX() * currentRotation.getCos()
				- transform.getY() * currentRotation.getSin();
		double ry = transform.getX() * currentRotation.getSin()
				+ transform.getY() * currentRotation.getCos();

		return new ChassisSpeeds(
			velocity.vxMetersPerSecond - velocity.omegaRadiansPerSecond * ry,
			velocity.vyMetersPerSecond + velocity.omegaRadiansPerSecond * rx,
			velocity.omegaRadiansPerSecond
		);
	}

}
