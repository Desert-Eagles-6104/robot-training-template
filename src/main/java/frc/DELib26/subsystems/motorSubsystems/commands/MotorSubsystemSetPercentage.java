package frc.DELib26.subsystems.motorSubsystems.commands;

import frc.DELib26.subsystems.motorSubsystems.motorBase.MotorSubsystemTalon;

import java.util.function.DoubleSupplier;

public class MotorSubsystemSetPercentage<T extends MotorSubsystemTalon> extends MotorDoubleSupplierCommand<T> {

	private final boolean ignoreSoftLimits;

	public MotorSubsystemSetPercentage(T motorSubsystem, DoubleSupplier valueSupplier) {
		this(motorSubsystem, valueSupplier, false);
	}

	public MotorSubsystemSetPercentage(T motorSubsystem, DoubleSupplier valueSupplier, boolean ignoreSoftLimits) {
		super(motorSubsystem, valueSupplier);
		this.ignoreSoftLimits = ignoreSoftLimits;
	}

	@Override
	public void execute() {
		this.motorSubsystem.setPercentOutput(this.valueSupplier.getAsDouble(), ignoreSoftLimits);
	}
}
