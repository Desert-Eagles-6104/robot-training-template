package frc.DELib26.motors;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.sim.TalonFXSimState.MotorType;
import edu.wpi.first.math.system.plant.DCMotor;
import frc.DELib26.util.RunMode;

import java.util.function.Consumer;

public class MasterMotorConstants extends MotorConstants {
	// TalonFX configuration for this motor
	private final TalonFXConfiguration talonFXConfiguration;

	public MasterMotorConstants(
			int id, String bus, boolean counterClockwisePositive, TalonFXConfiguration talonFXConfiguration) {
		this(id, bus, counterClockwisePositive, DCMotor.getKrakenX60(1), MotorType.KrakenX60, talonFXConfiguration);
	}

	public MasterMotorConstants(
			int id, String bus, boolean counterClockwisePositive, DCMotor motorSimulationSetting, MotorType motorSimType, TalonFXConfiguration talonFXConfiguration) {
		this(id, bus, counterClockwisePositive, motorSimulationSetting, motorSimType, talonFXConfiguration, config -> {});
	}

	public MasterMotorConstants(
			int id, String bus, boolean counterClockwisePositive, TalonFXConfiguration talonFXConfiguration, Consumer<TalonFXConfiguration> simConfigApplier) {
		this(id, bus, counterClockwisePositive, DCMotor.getKrakenX60(1), MotorType.KrakenX60, talonFXConfiguration, simConfigApplier);
	}

	public MasterMotorConstants(
			int id,
			String bus,
			boolean counterClockwisePositive,
			DCMotor motorSimulationSetting,
			MotorType motorSimType,
			TalonFXConfiguration talonFXConfiguration,
			Consumer<TalonFXConfiguration> simConfigApplier
	) {
		super(id, bus, counterClockwisePositive, motorSimulationSetting, motorSimType);
		if (RunMode.isSim()) {
			simConfigApplier.accept(talonFXConfiguration);
		}
		this.talonFXConfiguration = talonFXConfiguration;
		this.talonFXConfiguration.MotorOutput.Inverted = MotorConstants.toInvertedType(counterClockwisePositive);
	}

	public TalonFXConfiguration getTalonFXConfig() {
		return this.talonFXConfiguration;
	}
}
