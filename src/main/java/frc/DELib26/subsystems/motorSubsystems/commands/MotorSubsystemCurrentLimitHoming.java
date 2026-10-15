package frc.DELib26.subsystems.motorSubsystems.commands;

import frc.DELib26.subsystems.motorSubsystems.currentTripDetectors.CurrentTripDetector;
import frc.DELib26.subsystems.motorSubsystems.motorBase.MotorSubsystemTalon;

import java.util.function.DoubleSupplier;


public class MotorSubsystemCurrentLimitHoming<T extends MotorSubsystemTalon> extends MotorSubsystemHoming<T> {
	private final CurrentTripDetector currentTripDetector;

	/**
	 * @param motorSubsystem  the motor to home
	 * @param percent open-loop output during homing (-1..1)
	 * @param dwellTimeSeconds time that homing condition must be met before homing is successful
	 * @param timeoutSeconds optional max duration; <=0 disables timeout
	 * @param currentTripDetector detector to use for current-based trip detection
	 */
	public MotorSubsystemCurrentLimitHoming(
		T motorSubsystem,
		DoubleSupplier percent,
		double dwellTimeSeconds,
		double timeoutSeconds,
		CurrentTripDetector currentTripDetector
	) {
		super(motorSubsystem, percent, dwellTimeSeconds, timeoutSeconds);
		this.currentTripDetector = currentTripDetector;
	}

	@Override
	public void initialize() {
		super.initialize();
		this.currentTripDetector.initialize();
	}

	@Override
	protected boolean isHomeConditionMet() {
		return this.currentTripDetector.update(this.motorSubsystem.getStatorCurrent());
	}
}
