package frc.DELib26.controller;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.simulation.DriverStationSim;
import frc.DELib26.controller.GenericController.ControllerInput;

import java.util.function.Function;

/** Class to control a simulated generic joystick. */
public class GenericSimController {
	/** GenericHID port. */
	protected final int m_port;

	protected final Function<ControllerInput, Integer> m_inputMapper;

	/**
	 * Constructs from a GenericController object.
	 *
	 * @param joystick controller to simulate
	 */
	public GenericSimController(GenericController joystick) {
		m_port = joystick.getHID().getPort();
		m_inputMapper = joystick::getInputID;
		ControllerType type = joystick.getControllerType();
		setAxisCount(type.axisCount);
		setButtonCount(type.buttonCount);
		setPOVCount(type.povCount);
	}

	/** Updates joystick data so that new values are visible to the user program. */
	public void notifyNewData() {
		DriverStationSim.notifyNewData();
	}

	/**
	 * Set the value of a given button.
	 *
	 * @param button the button to set
	 * @param value  the new value
	 */
	public void setRawButton(int button, boolean value) {
		DriverStationSim.setJoystickButton(m_port, button, value);
	}

	/**
	 * Set the value of a given axis.
	 *
	 * @param axis  the axis to set
	 * @param value the new value
	 */
	public void setRawAxis(int axis, double value) {
		DriverStationSim.setJoystickAxis(m_port, axis, value);
	}

	/**
	 * Set the value of a given POV.
	 *
	 * @param pov   the POV to set
	 * @param value the new value
	 */
	public void setPOV(int pov, int value) {
		DriverStationSim.setJoystickPOV(m_port, pov, value);
	}

	/**
	 * Set the value of the default POV (port 0).
	 *
	 * @param value the new value
	 */
	public void setPOV(int value) {
		setPOV(0, value);
	}

	/**
	 * Set the axis count of this device.
	 *
	 * @param count the new axis count
	 */
	public void setAxisCount(int count) {
		DriverStationSim.setJoystickAxisCount(m_port, count);
	}

	/**
	 * Set the POV count of this device.
	 *
	 * @param count the new POV count
	 */
	public void setPOVCount(int count) {
		DriverStationSim.setJoystickPOVCount(m_port, count);
	}

	/**
	 * Set the button count of this device.
	 *
	 * @param count the new button count
	 */
	public void setButtonCount(int count) {
		DriverStationSim.setJoystickButtonCount(m_port, count);
	}

	/**
	 * Set the type of this device.
	 *
	 * @param type the new device type
	 */
	public void setType(GenericHID.HIDType type) {
		DriverStationSim.setJoystickType(m_port, type.value);
	}

	/**
	 * Set the name of this device.
	 *
	 * @param name the new device name
	 */
	public void setName(String name) {
		DriverStationSim.setJoystickName(m_port, name);
	}

	/**
	 * Set the type of the provided axis channel.
	 *
	 * @param axis the axis
	 * @param type the type
	 */
	public void setAxisType(int axis, int type) {
		DriverStationSim.setJoystickAxisType(m_port, axis, type);
	}

	/**
	 * Read the output of a button.
	 *
	 * @param outputNumber the button number
	 * @return the value of the button (true = pressed)
	 */
	public boolean getOutput(int outputNumber) {
		long outputs = getOutputs();
		return (outputs & (1L << (outputNumber - 1))) != 0;
	}

	/**
	 * Get the encoded 16-bit integer that passes button values.
	 *
	 * @return the button values
	 */
	public long getOutputs() {
		return DriverStationSim.getJoystickOutputs(m_port);
	}

	/**
	 * Get the joystick rumble.
	 *
	 * @param type the rumble to read
	 * @return the rumble value
	 */
	public double getRumble(GenericHID.RumbleType type) {
		int value = DriverStationSim.getJoystickRumble(m_port, type == GenericHID.RumbleType.kLeftRumble ? 0 : 1);
		return value / 65535.0;
	}

	/**
	 * Change the left X value of the controller's joystick.
	 *
	 * @param value the new value
	 */
	public void setLeftX(double value) {
		setRawAxis(m_inputMapper.apply(ControllerInput.LEFT_X_AXIS), value);
	}

	/**
	 * Change the right X value of the controller's joystick.
	 *
	 * @param value the new value
	 */
	public void setRightX(double value) {
		setRawAxis(m_inputMapper.apply(ControllerInput.RIGHT_X_AXIS), value);
	}

	/**
	 * Change the left Y value of the controller's joystick.
	 *
	 * @param value the new value
	 */
	public void setLeftY(double value) {
		setRawAxis(m_inputMapper.apply(ControllerInput.LEFT_Y_AXIS), value);
	}

	/**
	 * Change the right Y value of the controller's joystick.
	 *
	 * @param value the new value
	 */
	public void setRightY(double value) {
		setRawAxis(m_inputMapper.apply(ControllerInput.RIGHT_Y_AXIS), value);
	}

	/**
	 * Change the value of the left trigger axis on the controller.
	 *
	 * @param value the new value
	 */
	public void setLeftTriggerAxis(double value) {
		setRawAxis(m_inputMapper.apply(ControllerInput.LEFT_TRIGGER), value);
	}

	/**
	 * Change the value of the right trigger axis on the controller.
	 *
	 * @param value the new value
	 */
	public void setRightTriggerAxis(double value) {
		setRawAxis(m_inputMapper.apply(ControllerInput.RIGHT_TRIGGER), value);
	}

	/**
	 * Change the value of the down button on the controller.
	 *
	 * @param value the new value
	 */
	public void setDownButton(boolean value) {
		setRawButton(m_inputMapper.apply(ControllerInput.SOUTH), value);
	}

	/**
	 * Change the value of the right button on the controller.
	 *
	 * @param value the new value
	 */
	public void setRightButton(boolean value) {
		setRawButton(m_inputMapper.apply(ControllerInput.EAST), value);
	}

	/**
	 * Change the value of the left button on the controller.
	 *
	 * @param value the new value
	 */
	public void setLeftButton(boolean value) {
		setRawButton(m_inputMapper.apply(ControllerInput.WEST), value);
	}

	/**
	 * Change the value of the up button on the controller.
	 *
	 * @param value the new value
	 */
	public void setUpButton(boolean value) {
		setRawButton(m_inputMapper.apply(ControllerInput.NORTH), value);
	}

	/**
	 * Change the value of the left bumper button on the controller.
	 *
	 * @param value the new value
	 */
	public void setLeftBumperButton(boolean value) {
		setRawButton(m_inputMapper.apply(ControllerInput.LEFT_BUMPER), value);
	}

	/**
	 * Change the value of the right bumper button on the controller.
	 *
	 * @param value the new value
	 */
	public void setRightBumperButton(boolean value) {
		setRawButton(m_inputMapper.apply(ControllerInput.RIGHT_BUMPER), value);
	}

	/**
	 * Change the value of the view button on the controller.
	 *
	 * @param value the new value
	 */
	public void setViewButton(boolean value) {
		setRawButton(m_inputMapper.apply(ControllerInput.VIEW_BUTTON), value);
	}

	/**
	 * Change the value of the options button on the controller.
	 *
	 * @param value the new value
	 */
	public void setOptionsButton(boolean value) {
		setRawButton(m_inputMapper.apply(ControllerInput.OPTIONS_BUTTON), value);
	}

	/**
	 * Change the value of the left stick button on the controller.
	 *
	 * @param value the new value
	 */
	public void setLeftStickButton(boolean value) {
		setRawButton(m_inputMapper.apply(ControllerInput.LEFT_STICK_BUTTON), value);
	}

	/**
	 * Change the value of the right stick button on the controller.
	 *
	 * @param value the new value
	 */
	public void setRightStickButton(boolean value) {
		setRawButton(m_inputMapper.apply(ControllerInput.RIGHT_STICK_BUTTON), value);
	}
}
