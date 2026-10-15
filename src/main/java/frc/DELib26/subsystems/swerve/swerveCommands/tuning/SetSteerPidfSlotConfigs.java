package frc.DELib26.subsystems.swerve.swerveCommands.tuning;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.Slot1Configs;
import com.ctre.phoenix6.configs.Slot2Configs;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.DELib26.subsystems.swerve.SwerveSubsystem;

import java.util.function.Consumer;
import java.util.function.Supplier;

public class SetSteerPidfSlotConfigs extends InstantCommand {

	private SetSteerPidfSlotConfigs(SwerveSubsystem swerve, Consumer<TalonFXConfigurator> applier) {
		super(() -> swerve.forEachSteerMotor(m -> applier.accept(m.getConfigurator())));
	}

	public static SetSteerPidfSlotConfigs slot0(SwerveSubsystem swerve, Supplier<Slot0Configs> cfg) {
		return new SetSteerPidfSlotConfigs(swerve, c -> c.apply(cfg.get()));
	}

	public static SetSteerPidfSlotConfigs slot1(SwerveSubsystem swerve, Supplier<Slot1Configs> cfg) {
		return new SetSteerPidfSlotConfigs(swerve, c -> c.apply(cfg.get()));
	}

	public static SetSteerPidfSlotConfigs slot2(SwerveSubsystem swerve, Supplier<Slot2Configs> cfg) {
		return new SetSteerPidfSlotConfigs(swerve, c -> c.apply(cfg.get()));
	}
}
