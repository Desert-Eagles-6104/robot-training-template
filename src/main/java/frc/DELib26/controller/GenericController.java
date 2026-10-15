package frc.DELib26.controller;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.GenericHID.HIDType;
import edu.wpi.first.wpilibj.event.EventLoop;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.button.CommandGenericHID;
import edu.wpi.first.wpilibj2.command.button.Trigger;

import java.util.Map;

/**
 * An interface that defines a generic controller with methods to create button
 * and axis triggers,
 * get axis values, and set rumble. This interface can be implemented by
 * specific controller types
 * (e.g., Xbox, PS5) to provide a consistent way to interact with different
 * controllers.
 * Any methods that aren't universal for both controllers were removed.
 */
public class GenericController extends CommandGenericHID{
	private Map<ControllerInput, Integer> inputIDMap;
	private ControllerType controllerType;

	/**
	 * Enum representing various controller inputs.
	 */
	public static enum ControllerInput {
		WEST,
		NORTH,
		EAST,
		SOUTH,

		LEFT_BUMPER,
		RIGHT_BUMPER,

		LEFT_STICK_BUTTON,
		RIGHT_STICK_BUTTON,

		OPTIONS_BUTTON,
		VIEW_BUTTON,

		LEFT_TRIGGER,
		RIGHT_TRIGGER,

		LEFT_X_AXIS,
		LEFT_Y_AXIS,
		RIGHT_X_AXIS,
		RIGHT_Y_AXIS,
	}

	public GenericController(int port, ControllerType controllerType) {
		super(port);
		this.setControllerType(controllerType);
	}

	public void setControllerType(ControllerType controllerType) {
		this.controllerType = controllerType;
		this.inputIDMap = controllerType.inputIDMap;
	}

	public void assignControllerType() {
		if (!this.isConnected()) {
			return;
		}
		HIDType type = super.getHID().getType();
		String name = super.getHID().getName();
		if (type == HIDType.kXInputGamepad && name.contains("Xbox")) {
			this.setControllerType(ControllerType.XBOX);
		} else if (type == HIDType.kHIDGamepad && name.contains("DualSense")) {
			this.setControllerType(ControllerType.PS5);
		} else {
			DriverStation.reportWarning("Unknown controller type or name: " + type + ", " + name, false);
		}
	}

	public ControllerType getControllerType() {
		return this.controllerType;
	}

	/**
	 * Checks if any of the stick axes are active (beyond the deadband).
	 *
	 * @param deadband the deadband to apply to all stick axes
	 * @return true if any stick axis is active
	 */
	public boolean areStickAxesActive(double deadband) {
		return this.areStickAxesActive(deadband, deadband);
	}

	/**
	 * Checks if any of the stick axes are active (beyond the deadband).
	 *
	 * @param leftStickDeadband the deadband to apply to the left stick axes
	 * @param rightStickDeadband the deadband to apply to the right stick axes
	 * @return true if any stick axis is active
	 */
	public boolean areStickAxesActive(double leftStickDeadband, double rightStickDeadband) {
		return anyActive(leftStickDeadband, getLeftX(), getLeftY())
				|| anyActive(rightStickDeadband, getRightX(), getRightY());
	}

	/**
	 * Checks if any of the left stick axes are active (beyond the deadband).
	 *
	 * @param leftStickDeadband the deadband to apply to the left stick axes
	 * @return true if any left stick axis is active
	 */
	public boolean isLeftStickActive(double leftStickDeadband) {
		return anyActive(leftStickDeadband, getLeftX(), getLeftY());
	}

	/**
	 * Checks if any of the right stick axes are active (beyond the deadband).
	 *
	 * @param rightStickDeadband the deadband to apply to the right stick axes
	 * @return true if any right stick axis is active
	 */
	public boolean isRightStickActive(double rightStickDeadband) {
		return anyActive(rightStickDeadband, getRightX(), getRightY());
	}

	/**
	 * Checks if any of the provided axes are active (beyond the deadband).
	 *
	 * @param deadband the deadband to apply to all axes
	 * @param axes the axes to check
	 * @return true if any axis is active
	 */
	private static boolean anyActive(double deadband, double... axes) {
		double db = Math.max(0.0, deadband);
		for (double a : axes) {
			if (Math.abs(a) > db) return true;
		}
		return false;
	}

	/**
	 * Gets the unique ID associated with a specific controller input.
	 *
	 * @param input The controller input to retrieve the ID for.
	 * @return The ID of the specified controller input.
	 */
	public int getInputID(ControllerInput input) {
		if (inputIDMap.containsKey(input)) {
			return inputIDMap.get(input);
		} else {
			throw new IllegalArgumentException("Input " + input + " not found in inputIDMap");
		}
	}

	/**
	 * Constructs a Trigger instance around the west button's digital signal.
	 *
	 * @return a Trigger instance representing the west button's digital signal
	 *         attached
	 *         to the {@link CommandScheduler#getDefaultButtonLoop() default
	 *         scheduler button loop}.
	 * @see #westButton(EventLoop)
	 */
	public Trigger westButton() {
		return super.button(this.getInputID(ControllerInput.WEST));
	}

	/**
	 * Constructs a Trigger instance around the west button's digital signal.
	 *
	 * @param loop the event loop instance to attach the event to.
	 * @return a Trigger instance representing the west button's digital signal
	 *         attached
	 *         to the given loop.
	 */
	public Trigger westButton(EventLoop loop) {
		return super.button(this.getInputID(ControllerInput.WEST), loop);
	}

	/**
	 * Constructs a Trigger instance around the south button's digital signal.
	 *
	 * @return a Trigger instance representing the south button's digital signal
	 *         attached
	 *         to the {@link CommandScheduler#getDefaultButtonLoop() default
	 *         scheduler button loop}.
	 * @see #southButton(EventLoop)
	 */
	public Trigger southButton() {
		return super.button(this.getInputID(ControllerInput.SOUTH));
	}

	/**
	 * Constructs a Trigger instance around the south button's digital signal.
	 *
	 * @param loop the event loop instance to attach the event to.
	 * @return a Trigger instance representing the south button's digital signal
	 *         attached
	 *         to the given loop.
	 */
	public Trigger southButton(EventLoop loop) {
		return super.button(this.getInputID(ControllerInput.SOUTH), loop);
	}

	/**
	 * Constructs a Trigger instance around the east button's digital signal.
	 *
	 * @return a Trigger instance representing the east button's digital signal
	 *         attached
	 *         to the {@link CommandScheduler#getDefaultButtonLoop() default
	 *         scheduler button loop}.
	 * @see #eastButton(EventLoop)
	 */
	public Trigger eastButton() {
		return super.button(this.getInputID(ControllerInput.EAST));
	}

	/**
	 * Constructs a Trigger instance around the east button's digital signal.
	 *
	 * @param loop the event loop instance to attach the event to.
	 * @return a Trigger instance representing the east button's digital signal
	 *         attached
	 *         to the given loop.
	 */
	public Trigger eastButton(EventLoop loop) {
		return super.button(this.getInputID(ControllerInput.EAST), loop);
	}

	/**
	 * Constructs a Trigger instance around the north button's digital signal.
	 *
	 * @return a Trigger instance representing the north button's digital signal
	 *         attached
	 *         to the {@link CommandScheduler#getDefaultButtonLoop() default
	 *         scheduler button loop}.
	 * @see #northButton(EventLoop)
	 */
	public Trigger northButton() {
		return super.button(this.getInputID(ControllerInput.NORTH));
	}

	/**
	 * Constructs a Trigger instance around the north button's digital signal.
	 *
	 * @param loop the event loop instance to attach the event to.
	 * @return a Trigger instance representing the north button's digital signal
	 *         attached
	 *         to the given loop.
	 */
	public Trigger northButton(EventLoop loop) {
		return super.button(this.getInputID(ControllerInput.NORTH), loop);
	}

	/**
	 * Constructs a Trigger instance around the left bumper button's digital signal.
	 *
	 * @return a Trigger instance representing the left bumper button's digital
	 *         signal attached
	 *         to the {@link CommandScheduler#getDefaultButtonLoop() default
	 *         scheduler button loop}.
	 * @see #leftBumper(EventLoop)
	 */
	public Trigger leftBumper() {
		return super.button(this.getInputID(ControllerInput.LEFT_BUMPER));
	}

	/**
	 * Constructs a Trigger instance around the left bumper button's digital signal.
	 *
	 * @param loop the event loop instance to attach the event to.
	 * @return a Trigger instance representing the left bumper button's digital
	 *         signal attached
	 *         to the given loop.
	 */
	public Trigger leftBumper(EventLoop loop) {
		return super.button(this.getInputID(ControllerInput.LEFT_BUMPER), loop);
	}

	/**
	 * Constructs a Trigger instance around the right bumper button's digital
	 * signal.
	 *
	 * @return a Trigger instance representing the right bumper button's digital
	 *         signal attached
	 *         to the {@link CommandScheduler#getDefaultButtonLoop() default
	 *         scheduler button loop}.
	 * @see #rightBumper(EventLoop)
	 */
	public Trigger rightBumper() {
		return super.button(this.getInputID(ControllerInput.RIGHT_BUMPER));
	}

	/**
	 * Constructs a Trigger instance around the right bumper button's digital
	 * signal.
	 *
	 * @param loop the event loop instance to attach the event to.
	 * @return a Trigger instance representing the right bumper button's digital
	 *         signal attached
	 *         to the given loop.
	 */
	public Trigger rightBumper(EventLoop loop) {
		return super.button(this.getInputID(ControllerInput.RIGHT_BUMPER), loop);
	}

	/**
	 * Constructs a Trigger instance around the view button's digital signal.
	 *
	 * @return a Trigger instance representing the view button's digital signal
	 *         attached
	 *         to the {@link CommandScheduler#getDefaultButtonLoop() default
	 *         scheduler button loop}.
	 * @see #view(EventLoop)
	 */
	public Trigger view() {
		return super.button(this.getInputID(ControllerInput.VIEW_BUTTON));
	}

	/**
	 * Constructs a Trigger instance around the view button's digital signal.
	 *
	 * @param loop the event loop instance to attach the event to.
	 * @return a Trigger instance representing the view button's digital signal
	 *         attached
	 *         to the given loop.
	 */
	public Trigger view(EventLoop loop) {
		return super.button(this.getInputID(ControllerInput.VIEW_BUTTON), loop);
	}

	/**
	 * Constructs a Trigger instance around the options button's digital signal.
	 *
	 * @return a Trigger instance representing the options button's digital signal
	 *         attached
	 *         to the {@link CommandScheduler#getDefaultButtonLoop() default
	 *         scheduler button loop}.
	 * @see #options(EventLoop)
	 */
	public Trigger options() {
		return super.button(this.getInputID(ControllerInput.OPTIONS_BUTTON));
	}

	/**
	 * Constructs a Trigger instance around the options button's digital signal.
	 *
	 * @param loop the event loop instance to attach the event to.
	 * @return a Trigger instance representing the options button's digital signal
	 *         attached
	 *         to the given loop.
	 */
	public Trigger options(EventLoop loop) {
		return super.button(this.getInputID(ControllerInput.OPTIONS_BUTTON), loop);
	}

	/**
	 * Constructs a Trigger instance around the left stick button's digital signal.
	 *
	 * @return a Trigger instance representing the left stick button's digital
	 *         signal attached
	 *         to the {@link CommandScheduler#getDefaultButtonLoop() default
	 *         scheduler button loop}.
	 * @see #leftStick(EventLoop)
	 */
	public Trigger leftStick() {
		return super.button(this.getInputID(ControllerInput.LEFT_STICK_BUTTON));
	}

	/**
	 * Constructs a Trigger instance around the left stick button's digital signal.
	 *
	 * @param loop the event loop instance to attach the event to.
	 * @return a Trigger instance representing the left stick button's digital
	 *         signal attached
	 *         to the given loop.
	 */
	public Trigger leftStick(EventLoop loop) {
		return super.button(this.getInputID(ControllerInput.LEFT_STICK_BUTTON), loop);
	}

	/**
	 * Constructs a Trigger instance around the right stick button's digital signal.
	 *
	 * @return a Trigger instance representing the right stick button's digital
	 *         signal attached
	 *         to the {@link CommandScheduler#getDefaultButtonLoop() default
	 *         scheduler button loop}.
	 * @see #rightStick(EventLoop)
	 */
	public Trigger rightStick() {
		return super.button(this.getInputID(ControllerInput.RIGHT_STICK_BUTTON));
	}

	/**
	 * Constructs a Trigger instance around the right stick button's digital signal.
	 *
	 * @param loop the event loop instance to attach the event to.
	 * @return a Trigger instance representing the right stick button's digital
	 *         signal attached
	 *         to the given loop.
	 */
	public Trigger rightStick(EventLoop loop) {
		return super.button(this.getInputID(ControllerInput.RIGHT_STICK_BUTTON), loop);
	}

	/**
	 * Constructs a Trigger instance around the axis value of the left trigger. The
	 * returned
	 * trigger will be true when the axis value is greater than {@code threshold}.
	 *
	 * @param threshold the minimum axis value for the returned {@link Trigger} to
	 *                  be true. This value
	 *                  should be in the range [0, 1] where 0 is the unpressed state
	 *                  of the axis.
	 * @return a Trigger instance that is true when the left trigger's axis exceeds
	 *         the provided
	 *         threshold, attached to the
	 *         {@link CommandScheduler#getDefaultButtonLoop() default scheduler
	 *         button loop}.
	 */
	public Trigger leftTrigger(double threshold) {
		return super.axisGreaterThan(this.getInputID(ControllerInput.LEFT_TRIGGER), threshold);
	}

	/**
	 * Constructs a Trigger instance around the axis value of the left trigger. The
	 * returned trigger
	 * will be true when the axis value is greater than 0.5.
	 *
	 * @return a Trigger instance that is true when the left trigger's axis exceeds
	 *         0.5, attached to
	 *         the {@link CommandScheduler#getDefaultButtonLoop() default scheduler
	 *         button loop}.
	 */
	public Trigger leftTrigger() {
		return super.axisGreaterThan(this.getInputID(ControllerInput.LEFT_TRIGGER), 0.5);
	}

	/**
	 * Constructs a Trigger instance around the axis value of the right trigger. The
	 * returned
	 * trigger will be true when the axis value is greater than {@code threshold}.
	 *
	 * @param threshold the minimum axis value for the returned {@link Trigger} to
	 *                  be true. This value
	 *                  should be in the range [0, 1] where 0 is the unpressed state
	 *                  of the axis.
	 * @return a Trigger instance that is true when the right trigger's axis exceeds
	 *         the provided
	 *         threshold, attached to the
	 *         {@link CommandScheduler#getDefaultButtonLoop() default scheduler
	 *         button loop}.
	 */
	public Trigger rightTrigger(double threshold) {
		return super.axisGreaterThan(this.getInputID(ControllerInput.RIGHT_TRIGGER), threshold);
	}

	/**
	 * Constructs a Trigger instance around the axis value of the right trigger. The
	 * returned trigger
	 * will be true when the axis value is greater than 0.5.
	 *
	 * @return a Trigger instance that is true when the right trigger's axis exceeds
	 *         0.5, attached to
	 *         the {@link CommandScheduler#getDefaultButtonLoop() default scheduler
	 *         button loop}.
	 */
	public Trigger rightTrigger() {
		return super.axisGreaterThan(this.getInputID(ControllerInput.RIGHT_TRIGGER), 0.5);
	}

	/**
	 * Get the X axis value of left side of the controller. Right is positive.
	 *
	 * @return The axis value.
	 */
	public double getLeftX() {
		return super.getRawAxis(this.getInputID(ControllerInput.LEFT_X_AXIS));
	}

	/**
	 * Get the Y axis value of left side of the controller. Back is positive.
	 *
	 * @return The axis value.
	 */
	public double getLeftY() {
		return super.getRawAxis(this.getInputID(ControllerInput.LEFT_Y_AXIS));
	}

	/**
	 * Get the X axis value of right side of the controller. Right is positive.
	 *
	 * @return The axis value.
	 */
	public double getRightX() {
		return super.getRawAxis(this.getInputID(ControllerInput.RIGHT_X_AXIS));
	}

	/**
	 * Get the Y axis value of right side of the controller. Back is positive.
	 *
	 * @return The axis value.
	 */
	public double getRightY() {
		return super.getRawAxis(this.getInputID(ControllerInput.RIGHT_Y_AXIS));
	}

	/**
	 * Get the left trigger axis value of the controller. Note that this axis is
	 * bound to the
	 * range of [0, 1] as opposed to the usual [-1, 1].
	 *
	 * @return The axis value.
	 */
	public double getLeftTriggerAxis() {
		return super.getRawAxis(this.getInputID(ControllerInput.LEFT_TRIGGER));
	}

	/**
	 * Get the right trigger axis value of the controller. Note that this axis is
	 * bound to the
	 * range of [0, 1] as opposed to the usual [-1, 1].
	 *
	 * @return The axis value.
	 */
	public double getRightTriggerAxis() {
		return super.getRawAxis(this.getInputID(ControllerInput.RIGHT_TRIGGER));
	}
}
