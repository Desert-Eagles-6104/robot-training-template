package frc.DELib26.subsystems.motorSubsystems.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.DELib26.subsystems.motorSubsystems.motorBase.MotorSubsystemTalon;

import java.util.function.DoubleSupplier;

public abstract class MotorDoubleSupplierCommand<T extends MotorSubsystemTalon> extends Command {
	protected T motorSubsystem;
	protected DoubleSupplier valueSupplier;

	public MotorDoubleSupplierCommand(T motorSubsystem, DoubleSupplier valueSupplier) {
		this.valueSupplier = valueSupplier;
		this.motorSubsystem = motorSubsystem;
		addRequirements(motorSubsystem);
	}

	@Override
	public abstract void execute();

	@Override
	public void end(boolean interrupted) {
		this.motorSubsystem.disableMotors();
	}

	@Override
	public boolean isFinished() {
		return false;
	}
}
