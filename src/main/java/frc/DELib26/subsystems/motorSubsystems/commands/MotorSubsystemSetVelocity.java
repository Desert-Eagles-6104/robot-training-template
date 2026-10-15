package frc.DELib26.subsystems.motorSubsystems.commands;

import frc.DELib26.subsystems.motorSubsystems.motorBase.MotorSubsystemTalon;

import java.util.function.DoubleSupplier;

public class MotorSubsystemSetVelocity<T extends MotorSubsystemTalon> extends MotorDoubleSupplierCommand<T> {

	public MotorSubsystemSetVelocity(T motorSubsystem, DoubleSupplier valueSupplier) {
		super(motorSubsystem, valueSupplier);
	}

	@Override
	public void execute() {
		this.motorSubsystem.setVelocity(this.valueSupplier.getAsDouble());
	}
}
