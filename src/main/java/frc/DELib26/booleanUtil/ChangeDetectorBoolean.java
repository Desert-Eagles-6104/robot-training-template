package frc.DELib26.booleanUtil;

/**
 * A utility class for detecting changes in a value of any type.
 * This class is designed to monitor a value and determine if it has changed
 * since the last time it was checked. It is generic and can be used with any
 * type, but for boolean-specific use cases, it is recommended to use
 * {@code EdgeDetectorBoolean} for better clarity and functionality.
 *
 * @param <T> The type of the value to monitor for changes.
 */
public class ChangeDetectorBoolean<T> {
	private T lastValue;
	private final boolean omitNullValues;

	public ChangeDetectorBoolean(T initialValue) {
		this(initialValue, false);
	}

	public ChangeDetectorBoolean(T initialValue, boolean omitNullValues) {
		this.lastValue = initialValue;
		this.omitNullValues = omitNullValues;
	}

	/**
	 * Updates the internal state and returns true if the value has changed.
	 */
	public boolean hasChanged(T newValue) {
		if (omitNullValues && newValue == null) {
			return false;
		}
		boolean result = (lastValue == null && newValue != null)
						|| (lastValue != null && !lastValue.equals(newValue));
		lastValue = newValue;
		return result;
	}

	public T getLastValue() {
		return lastValue;
	}
}
