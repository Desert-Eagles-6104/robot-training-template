package frc.DELib26.motors;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;
import org.littletonrobotics.junction.AutoLog;

public interface MotorIO {

	@AutoLog
	class MotorIOInputs {
		public double positionUnits;
		public Angle mechanismPosition;
		public double velocityUnitsPerSec;
		public double mechanismVelocityRotPerSec;
		public double accelerationUnitsPerSecPerSec;
		public Current supplyCurrent;
		public Current statorCurrent;
		public double closedLoopError;
		public Voltage appliedVoltage;
		public Temperature temperature;
	}

	public void updateInputs(MotorIOInputs inputs);

	public void disableMotors();

	public void resetPosition(double positionRotations);

	public void setMotionMagicPositionSetpoint(double positionRotations);

	public void setMotionMagicVelocity(double velocityRPS);

	public void runCharacterization(Voltage volts);

	public void setPositionSetpoint(double positionRotations);

	public void setVelocity(double velocityRPS);

	public void setPercentOutput(double percent, boolean ignoreSoftLimits);

	public void setPIDSlot(int slot);

	public void controlForwardSoftLimit(boolean enableSoftLimit, double forwardSoftLimitThreshold);

	public void controlReverseSoftLimit(boolean enableSoftLimit, double reverseSoftLimitThreshold);

	public void setEnableAutosetPositionValue(boolean forward, boolean reverse);

	public void setNeutralMode(NeutralModeValue mode);

	public void applyConfig(TalonFXConfiguration config);

}
