package frc.DELib26.subsystems.motorSubsystems.commands;

import frc.DELib26.subsystems.motorSubsystems.motorBase.MotorSubsystemTalon;

import java.util.function.DoubleSupplier;

public class MotorSubsystemSetPosition<T extends MotorSubsystemTalon> extends MotorDoubleSupplierCommand<T> {

	public MotorSubsystemSetPosition(T motorSubsystemTalon, DoubleSupplier positionSupplier) {
		super(motorSubsystemTalon, positionSupplier);
	}

	@Override
	public void execute() {
		this.motorSubsystem.setPosition(this.valueSupplier.getAsDouble());
	}

	@Override
	public void end(boolean interrupted) {
		if (interrupted) {
			this.motorSubsystem.disableMotors();
		} else {
			this.motorSubsystem.setPosition(this.motorSubsystem.getPosition());
		}
	}

	@Override
	public boolean isFinished() {
		return this.motorSubsystem.isAtPositionSetpoint();
	}
}
