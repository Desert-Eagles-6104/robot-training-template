package frc.DELib26.booleanUtil;
/**
 * A utility class for detecting rising and falling edges of a boolean signal.
 */
public class EdgeDetectorBoolean {
	private boolean lastValue;

	public EdgeDetectorBoolean(boolean initialValue) {
		this.lastValue = initialValue;
	}

	/**
	 * Updates the internal state and returns true if a rising edge is detected.
	 */
	public boolean risingEdge(boolean newValue) {
		boolean result = !this.lastValue && newValue;
		this.lastValue = newValue;
		return result;
	}

	/**
	 * Updates the internal state and returns true if a falling edge is detected.
	 */
	public boolean fallingEdge(boolean newValue) {
		boolean result = this.lastValue && !newValue;
		this.lastValue = newValue;
		return result;
	}

	/**
	 * Updates the internal state and returns true if any edge (rising or falling) is detected.
	 */
	public boolean anyEdge(boolean newValue) {
		boolean result = this.lastValue != newValue;
		this.lastValue = newValue;
		return result;
	}

	public boolean getLastValue() {
		return this.lastValue;
	}
}
