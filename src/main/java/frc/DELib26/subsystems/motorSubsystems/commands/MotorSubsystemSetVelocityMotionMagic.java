package frc.DELib26.subsystems.motorSubsystems.commands;

import frc.DELib26.subsystems.motorSubsystems.motorBase.MotorSubsystemTalon;

import java.util.function.DoubleSupplier;

public class MotorSubsystemSetVelocityMotionMagic<T extends MotorSubsystemTalon> extends MotorDoubleSupplierCommand<T> {

	public MotorSubsystemSetVelocityMotionMagic(T motorSubsystem, DoubleSupplier valueSupplier) {
		super(motorSubsystem, valueSupplier);
	}

	@Override
	public void execute() {
		this.motorSubsystem.setMotionMagicVelocity(this.valueSupplier.getAsDouble());
	}
}
