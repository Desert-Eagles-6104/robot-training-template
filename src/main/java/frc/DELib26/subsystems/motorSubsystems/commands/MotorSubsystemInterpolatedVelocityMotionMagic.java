package frc.DELib26.subsystems.motorSubsystems.commands;

import frc.DELib26.subsystems.motorSubsystems.motorBase.MotorSubsystemTalon;

import java.util.function.DoubleSupplier;

public class MotorSubsystemInterpolatedVelocityMotionMagic<T extends MotorSubsystemTalon> extends MotorSubsystemInterpolatedVelocity<T> {

	public MotorSubsystemInterpolatedVelocityMotionMagic(T motorSubsystem, double[][] table, DoubleSupplier inputSupplier) {
		super(motorSubsystem, table, inputSupplier);
	}

	@Override
	public void execute() {
		double input = this.valueSupplier.getAsDouble();
		double velocity = this.interpolator.getInterpolatedValue(input);
		this.motorSubsystem.setMotionMagicVelocity(velocity);
	}
}
