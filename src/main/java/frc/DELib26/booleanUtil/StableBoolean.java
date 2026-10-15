package frc.DELib26.booleanUtil;

import edu.wpi.first.wpilibj.Timer;

import java.util.function.BooleanSupplier;

/** Returns true only if the input has been true continuously for at least timeThreshold seconds. */
public class StableBoolean implements BooleanSupplier {
	private double timeThreshold;
	private final Timer timer = new Timer();
	private boolean lastValue = false;

	public StableBoolean(double timeThreshold) {
		this.setTimeout(timeThreshold);
	}

	public void setTimeout(double timeThreshold) {
		if (timeThreshold < 0) throw new IllegalArgumentException("timeThreshold must be >= 0");
		this.timeThreshold = timeThreshold;
	}

	/** Clears state: behaves as if input has been false and timer not running. */
	public void reset() {
		lastValue = false;
		timer.stop();
		timer.reset();
	}

	/** Call every loop with the current boolean input. */
	public boolean update(boolean value) {
		// Rising edge
		if (!lastValue && value) {
			timer.reset();
			timer.start();
		}
		// Falling edge
		if (lastValue && !value) {
			timer.stop();
			timer.reset();
		}

		lastValue = value;
		return getAsBoolean();
	}

	@Override
	public boolean getAsBoolean() {
		return lastValue && timer.hasElapsed(timeThreshold);
	}
}
