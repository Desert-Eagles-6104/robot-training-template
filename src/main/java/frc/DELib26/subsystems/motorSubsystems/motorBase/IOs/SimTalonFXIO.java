package frc.DELib26.subsystems.motorSubsystems.motorBase.IOs;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.sim.ChassisReference;
import com.ctre.phoenix6.sim.TalonFXSimState;
import frc.DELib26.motors.MotorConstants;
import frc.DELib26.subsystems.motorSubsystems.motorBase.MotorSubsystemConfiguration;

public class SimTalonFXIO extends TalonFXIO {
	private final TalonFXSimState simState;

	//it needs to get motor consts brcuse if not it wont know witch motor it is
	public SimTalonFXIO(MotorSubsystemConfiguration subsystemConfig, MotorConstants motorConstants, TalonFX motor) {
		super(subsystemConfig, motorConstants, motor);
		this.simState = motor.getSimState();
		this.simState.setMotorType(motorConstants.getMotorSimType());
		this.simState.Orientation = motorConstants.counterClockwisePositive
				? ChassisReference.CounterClockwise_Positive
				: ChassisReference.Clockwise_Positive;
	}

	public double rotorPerMech(){
		return this.subsystemConfig.master.getTalonFXConfig().Feedback.SensorToMechanismRatio;
	}

	public TalonFXSimState getSimState() {
		return this.simState;
	}

}
