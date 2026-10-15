package frc.DELib26.util.geometry;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Translation2d;

public final record Bounds(
	double minX,
	double minY,
	double maxX,
	double maxY
){

	public boolean contains(Translation2d p) {
		return p.getX() >= minX && p.getX() <= maxX && p.getY() >= minY && p.getY() <= maxY;
	}

	public Bounds expand(double margin) {
		return new Bounds(
			minX - margin,
			minY - margin,
			maxX + margin,
			maxY + margin
		);
	}

	public Bounds shrink(double margin) {
		return new Bounds(
			minX + margin,
			minY + margin,
			maxX - margin,
			maxY - margin
		);
	}

	public Translation2d center() {
		return new Translation2d((minX + maxX) * 0.5, (minY + maxY) * 0.5);
	}

	public Translation2d clampPoint(Translation2d p) {
		return clampPoint(p, 0.0);
	}

	public Translation2d clampPoint(Translation2d p, double margin) {
		return new Translation2d(
			MathUtil.clamp(p.getX(), minX + margin, maxX - margin),
			MathUtil.clamp(p.getY(), minY + margin, maxY - margin)
		);
	}

	public double widthX() { return maxX - minX; }
	public double widthY() { return maxY - minY; }


	/**
	 * Returns true if the closed segment [a,b] intersects (or passes through) this Bounds.
	 * Robust + fast (slab method). Includes "touching edge" as intersection.
	 */
	public boolean intersectsSegment(Translation2d a, Translation2d b) {
		// If either endpoint is inside, definitely intersects
		if (contains(a) || contains(b)) return true;

		final double dx = b.getX() - a.getX();
		final double dy = b.getY() - a.getY();

		double t0 = 0.0;
		double tSegmentEnd = 1.0;

		// X slab
		if (Math.abs(dx) < 1e-12) {
			if (a.getX() < minX || a.getX() > maxX) return false;
		} else {
			double invDx = 1.0 / dx;
			double tx1 = (minX - a.getX()) * invDx;
			double tx2 = (maxX - a.getX()) * invDx;
			double tEnter = Math.min(tx1, tx2);
			double tExit  = Math.max(tx1, tx2);
			t0 = Math.max(t0, tEnter);
			tSegmentEnd = Math.min(tSegmentEnd, tExit);
			if (t0 > tSegmentEnd) return false;
		}

		// Y slab
		if (Math.abs(dy) < 1e-12) {
			if (a.getY() < minY || a.getY() > maxY) return false;
		} else {
			double invDy = 1.0 / dy;
			double ty1 = (minY - a.getY()) * invDy;
			double ty2 = (maxY - a.getY()) * invDy;
			double tEnter = Math.min(ty1, ty2);
			double tExit  = Math.max(ty1, ty2);
			t0 = Math.max(t0, tEnter);
			tSegmentEnd = Math.min(tSegmentEnd, tExit);
			if (t0 > tSegmentEnd) return false;
		}

		// There is overlap in parameter interval => segment intersects the box
		return tSegmentEnd >= 0.0 && t0 <= 1.0;
	}

}