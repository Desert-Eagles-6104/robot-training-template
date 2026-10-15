package frc.DELib26.booleanUtil;

import java.util.function.BooleanSupplier;

/**
 * A latched boolean.
 * Once set to true, it remains so until reset.
 */
public class LatchedBoolean implements BooleanSupplier {
	private boolean isLatched = false;
	private int requiredTrueCount;
	private int trueCount = 0; // not in a row

	public LatchedBoolean() {
		this(1);
	}

	public LatchedBoolean(int trueCountThreshold) {
		if (trueCountThreshold < 1) {
			throw new IllegalArgumentException("trueCountThreshold must be at least 1");
		}
		this.requiredTrueCount = trueCountThreshold;
	}

	public boolean update(boolean newValue) {
		if (newValue) {
			this.trueCount++;
		}

		if (this.trueCount > this.requiredTrueCount) {
			return this.isLatched = true;
		} else {
			return false;
		}
	}

	public void reset() {
		this.isLatched = false;
		this.trueCount = 0;
	}

	@Override
	public boolean getAsBoolean() {
		return this.isLatched;
	}
}
