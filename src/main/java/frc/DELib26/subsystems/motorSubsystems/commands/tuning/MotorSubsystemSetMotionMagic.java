package frc.DELib26.subsystems.motorSubsystems.commands.tuning;

import com.ctre.phoenix6.configs.MotionMagicConfigs;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.DELib26.subsystems.motorSubsystems.motorBase.MotorSubsystemTalon;

import java.util.function.Supplier;

public class MotorSubsystemSetMotionMagic extends InstantCommand{
	public MotorSubsystemSetMotionMagic(MotorSubsystemTalon motorSubsystem, Supplier<MotionMagicConfigs> config) {
		super(
			() -> motorSubsystem.setConfig(
				motorSubsystem.configuration.master
					.getTalonFXConfig()
						.withMotionMagic(
							config.get()
						)
			)
		);
	}
}
