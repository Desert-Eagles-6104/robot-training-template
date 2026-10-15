package frc.DELib26.subsystems.motorSubsystems.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.DELib26.interpolation.LinearInterpolator;
import frc.DELib26.subsystems.motorSubsystems.motorBase.MotorSubsystemTalon;

import java.util.function.DoubleSupplier;

public class MotorSubsystemInterpolatedVelocity<T extends MotorSubsystemTalon> extends Command {
	protected final T motorSubsystem;
	protected final LinearInterpolator interpolator;
	protected final DoubleSupplier valueSupplier;

	public MotorSubsystemInterpolatedVelocity(
			T motorSubsystem, double[][] table, DoubleSupplier valueSupplier) {
		this.motorSubsystem = motorSubsystem;
		this.interpolator = new LinearInterpolator(table);
		this.valueSupplier = valueSupplier;
		addRequirements(motorSubsystem);
	}

	@Override
	public void execute() {
		double input = this.valueSupplier.getAsDouble();
		double velocity = this.interpolator.getInterpolatedValue(input);
		this.motorSubsystem.setVelocity(velocity);
	}

	@Override
	public void end(boolean interrupted) {
		motorSubsystem.disableMotors();
	}

	@Override
	public boolean isFinished() {
		return false;
	}
}
