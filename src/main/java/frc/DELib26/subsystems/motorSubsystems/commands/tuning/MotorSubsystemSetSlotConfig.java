package frc.DELib26.subsystems.motorSubsystems.commands.tuning;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.Slot1Configs;
import com.ctre.phoenix6.configs.Slot2Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.DELib26.subsystems.motorSubsystems.motorBase.MotorSubsystemTalon;

import java.util.function.Supplier;

public class MotorSubsystemSetSlotConfig extends InstantCommand {

	@FunctionalInterface
	public interface SlotApplier {
		TalonFXConfiguration apply(TalonFXConfiguration cfg);
	}

	public MotorSubsystemSetSlotConfig(MotorSubsystemTalon motorSubsystem, SlotApplier applier) {
		super(() -> motorSubsystem.setConfig(
			applier.apply(motorSubsystem.configuration.master.getTalonFXConfig())
		), motorSubsystem);
	}

	public static SlotApplier slot0(Supplier<Slot0Configs> s) {
		return cfg -> cfg.withSlot0(s.get());
	}

	public static SlotApplier slot1(Supplier<Slot1Configs> s) {
		return cfg -> cfg.withSlot1(s.get());
	}

	public static SlotApplier slot2(Supplier<Slot2Configs> s) {
		return cfg -> cfg.withSlot2(s.get());
	}
}
