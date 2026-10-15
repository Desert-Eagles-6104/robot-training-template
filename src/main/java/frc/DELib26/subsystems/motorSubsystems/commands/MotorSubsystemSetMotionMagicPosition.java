package frc.DELib26.subsystems.motorSubsystems.commands;

import frc.DELib26.subsystems.motorSubsystems.motorBase.MotorSubsystemTalon;

import java.util.function.DoubleSupplier;

public class MotorSubsystemSetMotionMagicPosition<T extends MotorSubsystemTalon> extends MotorSubsystemSetPosition<T> {

	public MotorSubsystemSetMotionMagicPosition(T motorSubsystem, DoubleSupplier positionSupplier) {
		super(motorSubsystem, positionSupplier);
	}

	@Override
	public void execute() {
		this.motorSubsystem.setMotionMagicPosition(this.valueSupplier.getAsDouble());
	}

	@Override
	public void end(boolean interrupted) {
		if (interrupted) {
			this.motorSubsystem.disableMotors();
		} else {
			this.motorSubsystem.setMotionMagicPosition(this.motorSubsystem.getPosition());
		}
	}

}
