package frc.DELib26.motors;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import frc.DELib26.CANBus.CANBusRegistry;
import frc.DELib26.util.PhoneixUtil;

public class MotorFactory {

	public static TalonFX createTalonFX(MasterMotorConstants motorConstants) {
		TalonFX talon = new TalonFX(motorConstants.id, CANBusRegistry.get(motorConstants.bus));
		talon.clearStickyFaults();
		PhoneixUtil.checkErrorAndRetry(() -> talon.getConfigurator().apply(motorConstants.getTalonFXConfig()));
		return talon;
	}

	public static TalonFX createSlaveTalon(MotorConstants motorConstants, int masterId, boolean followMasterDirection) {
		MotorAlignmentValue alignmentValue = getAlignmentValue(followMasterDirection);
		TalonFX talon = new TalonFX(motorConstants.id, CANBusRegistry.get(motorConstants.bus));
		talon.clearStickyFaults();
		PhoneixUtil.checkErrorAndRetry(() -> talon.setControl(new Follower(masterId, alignmentValue)));
		return talon;
	}

	public static TalonFX createSlaveTalon(MotorConstants motorConstants, MasterMotorConstants masterConstants) {
		MotorAlignmentValue alignmentValue = getAlignmentValue(
				masterConstants.counterClockwisePositive == motorConstants.counterClockwisePositive
			);
		TalonFX talon = new TalonFX(motorConstants.id, CANBusRegistry.get(motorConstants.bus));
		talon.clearStickyFaults();
		PhoneixUtil.checkErrorAndRetry(() -> talon.setControl(new Follower(masterConstants.id, alignmentValue)));
		return talon;
	}

	public static TalonFX createSlaveTalonAndApplyConfig(MotorConstants motorConstants, MasterMotorConstants masterConstants) {
		TalonFX talon = createSlaveTalon(motorConstants, masterConstants);
		TalonFXConfiguration slaveConfig = makeFollowerConfigFromMaster(masterConstants.getTalonFXConfig());
		PhoneixUtil.checkErrorAndRetry(() -> talon.getConfigurator().apply(slaveConfig));
		return talon;
	}


	public static TalonFX[] createSlaveTalonsAndApplyConfig(MotorConstants[] followerConstants, MasterMotorConstants masterConstants) {
		TalonFX[] arr = new TalonFX[followerConstants.length];
		for (int i = 0; i < followerConstants.length; i++) {
			MotorConstants sc = followerConstants[i];
			TalonFX fx = createSlaveTalonAndApplyConfig(sc, masterConstants);
			fx.optimizeBusUtilization();
			arr[i] = fx;
		}
		return arr;
	}

	public static MotorAlignmentValue getAlignmentValue(boolean isAligned) {
		if (isAligned) {
			return MotorAlignmentValue.Aligned;
		} else {
			return MotorAlignmentValue.Opposed;
		}
	}

	public static TalonFXConfiguration makeFollowerConfigFromMaster(TalonFXConfiguration masterCfg) {
		TalonFXConfiguration cfg = new TalonFXConfiguration();

		cfg.Feedback = masterCfg.Feedback;
		cfg.MotorOutput = masterCfg.MotorOutput;
		cfg.CurrentLimits = masterCfg.CurrentLimits;
		cfg.Voltage = masterCfg.Voltage;
		cfg.OpenLoopRamps = masterCfg.OpenLoopRamps;
		cfg.ClosedLoopRamps = masterCfg.ClosedLoopRamps;

		return cfg;
	}



}
