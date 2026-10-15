package frc.DELib26.booleanUtil;

/**
 * A toggle boolean that flips its state on each rising edge of the input.
 */
public class ToggleBoolean {
	private final EdgeDetectorBoolean edge = new EdgeDetectorBoolean(false);
	private boolean state = false;

	public boolean update(boolean input) {
		if (edge.risingEdge(input)) state = !state;
		return state;
	}
}
