package frc.DELib26.controller;

import frc.DELib26.controller.GenericController.ControllerInput;

import java.util.Map;

public enum ControllerType {
	XBOX(ControllerMappings.XBOX_MAPPING, 10),
	PS5(ControllerMappings.PS5_MAPPING, 14);

	public final Map<ControllerInput, Integer> inputIDMap;
	public final int buttonCount;
	public final int axisCount;
	public final int povCount;

	ControllerType(Map<ControllerInput, Integer> inputIDMap, int buttonCount) {
		this.inputIDMap = inputIDMap;
		this.buttonCount = buttonCount;
		this.axisCount = 6;
		this.povCount = 1;
	}

	public GenericController buildController(int port) {
		return new GenericController(port, this);
	}
}
