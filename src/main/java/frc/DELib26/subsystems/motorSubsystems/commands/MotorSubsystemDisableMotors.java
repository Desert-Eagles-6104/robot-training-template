package frc.DELib26.subsystems.motorSubsystems.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.DELib26.subsystems.motorSubsystems.motorBase.MotorSubsystemTalon;

public class MotorSubsystemDisableMotors<T extends MotorSubsystemTalon> extends InstantCommand {
	private T motorSubsystem;

	public MotorSubsystemDisableMotors(T motorSubsystem) {
		this.motorSubsystem = motorSubsystem;
		addRequirements(motorSubsystem);
	}

	@Override
	public void initialize() {
		this.motorSubsystem.disableMotors();
	}
}
