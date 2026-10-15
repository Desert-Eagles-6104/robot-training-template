package frc.DELib26.controller;

import edu.wpi.first.wpilibj.PS5Controller;
import edu.wpi.first.wpilibj.XboxController;
import frc.DELib26.controller.GenericController.ControllerInput;

import java.util.HashMap;
import java.util.Map;

public class ControllerMappings {
	public static final Map<ControllerInput, Integer> XBOX_MAPPING;
	public static final Map<ControllerInput, Integer> PS5_MAPPING;

	static {
		XBOX_MAPPING = new HashMap<>();
		XBOX_MAPPING.put(ControllerInput.WEST, XboxController.Button.kX.value);
		XBOX_MAPPING.put(ControllerInput.NORTH, XboxController.Button.kY.value);
		XBOX_MAPPING.put(ControllerInput.EAST, XboxController.Button.kB.value);
		XBOX_MAPPING.put(ControllerInput.SOUTH, XboxController.Button.kA.value);

		XBOX_MAPPING.put(ControllerInput.LEFT_BUMPER, XboxController.Button.kLeftBumper.value);
		XBOX_MAPPING.put(ControllerInput.RIGHT_BUMPER, XboxController.Button.kRightBumper.value);

		XBOX_MAPPING.put(ControllerInput.LEFT_STICK_BUTTON, XboxController.Button.kLeftStick.value);
		XBOX_MAPPING.put(ControllerInput.RIGHT_STICK_BUTTON, XboxController.Button.kRightStick.value);

		XBOX_MAPPING.put(ControllerInput.OPTIONS_BUTTON, XboxController.Button.kStart.value);
		XBOX_MAPPING.put(ControllerInput.VIEW_BUTTON, XboxController.Button.kBack.value);

		XBOX_MAPPING.put(ControllerInput.LEFT_TRIGGER, XboxController.Axis.kLeftTrigger.value);
		XBOX_MAPPING.put(ControllerInput.RIGHT_TRIGGER, XboxController.Axis.kRightTrigger.value);

		XBOX_MAPPING.put(ControllerInput.LEFT_X_AXIS, XboxController.Axis.kLeftX.value);
		XBOX_MAPPING.put(ControllerInput.LEFT_Y_AXIS, XboxController.Axis.kLeftY.value);
		XBOX_MAPPING.put(ControllerInput.RIGHT_X_AXIS, XboxController.Axis.kRightX.value);
		XBOX_MAPPING.put(ControllerInput.RIGHT_Y_AXIS, XboxController.Axis.kRightY.value);

		PS5_MAPPING = new HashMap<>();
		PS5_MAPPING.put(ControllerInput.WEST, PS5Controller.Button.kSquare.value);
		PS5_MAPPING.put(ControllerInput.NORTH, PS5Controller.Button.kTriangle.value);
		PS5_MAPPING.put(ControllerInput.EAST, PS5Controller.Button.kCircle.value);
		PS5_MAPPING.put(ControllerInput.SOUTH, PS5Controller.Button.kCross.value);

		PS5_MAPPING.put(ControllerInput.LEFT_BUMPER, PS5Controller.Button.kL1.value);
		PS5_MAPPING.put(ControllerInput.RIGHT_BUMPER, PS5Controller.Button.kR1.value);

		PS5_MAPPING.put(ControllerInput.LEFT_STICK_BUTTON, PS5Controller.Button.kL3.value);
		PS5_MAPPING.put(ControllerInput.RIGHT_STICK_BUTTON, PS5Controller.Button.kR3.value);

		PS5_MAPPING.put(ControllerInput.OPTIONS_BUTTON, PS5Controller.Button.kOptions.value);
		PS5_MAPPING.put(ControllerInput.VIEW_BUTTON, PS5Controller.Button.kCreate.value);

		PS5_MAPPING.put(ControllerInput.LEFT_TRIGGER, PS5Controller.Axis.kL2.value);
		PS5_MAPPING.put(ControllerInput.RIGHT_TRIGGER, PS5Controller.Axis.kR2.value);

		PS5_MAPPING.put(ControllerInput.LEFT_X_AXIS, PS5Controller.Axis.kLeftX.value);
		PS5_MAPPING.put(ControllerInput.LEFT_Y_AXIS, PS5Controller.Axis.kLeftY.value);
		PS5_MAPPING.put(ControllerInput.RIGHT_X_AXIS, PS5Controller.Axis.kRightX.value);
		PS5_MAPPING.put(ControllerInput.RIGHT_Y_AXIS, PS5Controller.Axis.kRightY.value);
	}
}
