package frc.DELib26.util;

import com.ctre.phoenix6.Utils;

/**
 * A utility class for calculating the time difference (delta) between successive calls
 * to the {@link #update()} method or after a {@link #reset()}.
 * <p>
 * This class uses a timestamp source (e.g., {@code Utils.getCurrentTimeSeconds()}) to
 * measure elapsed time in seconds.
 * </p>
 *
 * <p>Usage:</p>
 * <pre>
 * TimestampDeltaCalculator deltaCalculator = new TimestampDeltaCalculator();
 * double deltaTime = deltaCalculator.update(); // Get time delta since initialization
 * deltaCalculator.reset(); // Reset the timestamp
 * </pre>
 */
public class TimestampDeltaCalculator {
	private double lastTimeSeconds;

	/**
	 * Constructs a new {@code TimestampDeltaCalculator} and initializes the
	 * internal timestamp to the current time in seconds.
	 */
	public TimestampDeltaCalculator() {
		this.lastTimeSeconds = Utils.getCurrentTimeSeconds();
	}

	/**
	 * Resets the internal timestamp to the current time in seconds.
	 */
	public void reset() {
		this.lastTimeSeconds = Utils.getCurrentTimeSeconds();
	}

	/**
	 * Updates the internal timestamp and calculates the time difference (delta)
	 * in seconds since the last update or reset.
	 *
	 * @return The time difference (delta) in seconds.
	 */
	public double update() {
		double currentTimeSeconds = Utils.getCurrentTimeSeconds();
		double deltaTime = currentTimeSeconds - this.lastTimeSeconds;
		this.lastTimeSeconds = currentTimeSeconds;
		return deltaTime;
	}

	public double calculateDeltaTime() {
		double currentTimeSeconds = Utils.getCurrentTimeSeconds();
		double deltaTime = currentTimeSeconds - this.lastTimeSeconds;
		return deltaTime;
	}
}
