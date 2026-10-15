package frc.DELib26.subsystems.motorSubsystems.currentTripDetectors;

import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.wpilibj.Timer;
import frc.DELib26.booleanUtil.StableBoolean;

/**
 * Abstract current-trip detector with shared lifecycle:
 *  - Moving-average filtering (window in SAMPLES)
 *  - Startup ignore window (sec)
 *  - Debounce (sec)
 *  - Optional absolute timeout (sec)
 *
 * Usage: call {@link #initialize()} once, then {@link #update(Current)} each cycle with
 * the latest current sample (amps). If {@code timeoutSeconds > 0}, {@code update()} will
 * return {@code true} once the timeout elapses (caller may treat as "give up/exit").
 *
 * Note: movingAvgWindowSamples is in samples, not seconds. With a 20 ms loop,
 * 10 samples ≈ 0.20 s.
 */
public abstract class CurrentTripDetector {

	protected static abstract class CurrentTripDetectorConfig {
		/**
		 * moving-average window length (>=1)
		 */
		public int movingAvgWindowSamples = 10;
		/**
		 * startup ignore window (sec)
		 */
		public double startupIgnoreSeconds = 0.1;
		/**
		 * require the condition to persist this long before reporting true
		 */
		public double debounceSeconds = 0;
	}

	private final LinearFilter filter;
	private final double startupIgnoreSeconds;

	private final Timer startupTimer = new Timer();

	private final StableBoolean stable;

	private double lastFiltered;

	protected CurrentTripDetector(CurrentTripDetectorConfig config) {

		if (config.movingAvgWindowSamples < 1) throw new IllegalArgumentException("movingAvgWindowSamples must be >= 1");
		if (config.startupIgnoreSeconds < 0) throw new IllegalArgumentException("startupIgnoreSeconds must be >= 0");
		if (config.debounceSeconds < 0) throw new IllegalArgumentException("debounceSeconds must be >= 0");

		this.filter = LinearFilter.movingAverage(config.movingAvgWindowSamples);
		this.startupIgnoreSeconds = config.startupIgnoreSeconds;
		this.stable = new StableBoolean(config.debounceSeconds);
	}

	/** One-time start/reset before calling update(). */
	public void initialize() {
		filter.reset();
		this.lastFiltered = Double.NaN;
		startupTimer.restart();
		this.stable.reset();
	}

	/** Alias for initialize(). */
	public void reset() { initialize(); }

	/**
	 * Feed one current sample (amps). Returns true once tripped (debounced) or timed out.
	 */
	public final boolean update(Current current) {
		// Robustify: clamp/abs input (defend against signed or NaN)
		double sample = Math.abs(current.in(Units.Amps));
		if (Double.isNaN(sample) || Double.isInfinite(sample)) {
			return false;
		}

		double filtered = filter.calculate(sample);
		// Init lastFiltered on first call
		if (Double.isNaN(lastFiltered)) {
			lastFiltered = filtered;
		}

		// Ignore during startup window
		if (startupTimer.get() < startupIgnoreSeconds) {
			lastFiltered = filtered;
			return false;
		}

		boolean condition = this.testCondition(filtered, lastFiltered);
		lastFiltered = filtered;

		// Debounce
		return stable.update(condition);
	}

	/** Implement the specific trip condition using filtered and previous-filtered current (amps). */
	protected abstract boolean testCondition(double filteredAmps, double prevFilteredAmps);

	public boolean isTripped() {
		return stable.getAsBoolean();
	}

	/** Last filtered current value (amps), for logging/tuning. */
	public final double getLastFilteredCurrent() { return lastFiltered; }

	/** True iff the startup-ignore window has elapsed (detector is "armed"). */
	public final boolean isArmed() { return startupTimer.get() >= startupIgnoreSeconds; }
}