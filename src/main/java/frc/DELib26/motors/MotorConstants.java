package frc.DELib26.motors;

import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.sim.TalonFXSimState.MotorType;
import edu.wpi.first.math.system.plant.DCMotor;
import frc.DELib26.CANBus.CanDeviceConstants;

public class MotorConstants extends CanDeviceConstants {
	public final boolean counterClockwisePositive;
	protected final DCMotor motorSimulationSetting;
	protected final MotorType motorSimType;

	public MotorConstants(
			int id,
			String bus,
			boolean counterClockwisePositive) {
		this(id, bus, counterClockwisePositive, DCMotor.getKrakenX60Foc(1), MotorType.KrakenX60);
	}

	public MotorConstants(
			int id,
			String bus,
			boolean counterClockwisePositive,
			DCMotor motorSimulationSetting,
			MotorType motorSimType
		) {
		super(id, bus);
		this.counterClockwisePositive = counterClockwisePositive;
		this.motorSimulationSetting = motorSimulationSetting;
		this.motorSimType = motorSimType;
	}

	public DCMotor getMotorSimulationSetting() {
		return this.motorSimulationSetting;
	}

	public MotorType getMotorSimType() {
		return this.motorSimType;
	}

	/**
	 * for talonFX
	 *
	 * @param counterClockwisePositive true if counter clockwise is positive
	 */
	public static InvertedValue toInvertedType(boolean counterClockwisePositive) {
		if (counterClockwisePositive) {
			return InvertedValue.CounterClockwise_Positive;
		}
		return InvertedValue.Clockwise_Positive;
	}
}
