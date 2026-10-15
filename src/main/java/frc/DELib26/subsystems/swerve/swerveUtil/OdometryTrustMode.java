package frc.DELib26.subsystems.swerve.swerveUtil;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;

/**
 * Represents different modes for how much we trust odometry.
 * x - Expected deviation in meters (e.g., 1 means -1 < x < 1 in position).
 * y - Expected deviation in meters.
 * theta - Expected deviation in radians.
 */
public enum OdometryTrustMode {
	NORMAL(VecBuilder.fill(0.1, 0.1, 0.1)),
	/**
	 * Low standard deviations when disabled, as odometry is more reliable.
	 */
	DISABLED(VecBuilder.fill(0.05, 0.5, 0.5)),
	/**
	 * Auto mode is typically more precise and predictable than normal.
	 */
	AUTO(VecBuilder.fill(0.01, 0.01, 0.005)),
	/**
	 * Skid mode with moderate standard deviations.
	 */
	SKID(VecBuilder.fill(0.5, 0.5, 0.2));

	private final Matrix<N3, N1> stateStdDevs;

	OdometryTrustMode(Matrix<N3, N1> stateStdDevs) {
		this.stateStdDevs = stateStdDevs;
	}

	public Matrix<N3, N1> getStateStdDevs() {
		return stateStdDevs;
	}
}
