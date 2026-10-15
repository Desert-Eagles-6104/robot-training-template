package frc.robot;

import frc.robot.constants.Constants;
import frc.robot.constants.Robot2026Constants;

import java.util.function.Supplier;

public enum RobotModel {
	ROBOT_2026(() -> new Robot2026Constants());

	private final Supplier<Constants> constants;
	private Constants cachedConstants;

	RobotModel(Supplier<Constants> constants) {
		this.constants = constants;
	}

	public Constants getConstants() {
		if (this.cachedConstants == null) {
			this.cachedConstants = this.constants.get();
		}
		return this.cachedConstants;
	}
}
