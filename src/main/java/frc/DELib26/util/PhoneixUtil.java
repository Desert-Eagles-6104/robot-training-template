package frc.DELib26.util;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.CANrangeConfiguration;
import com.ctre.phoenix6.configs.HardwareLimitSwitchConfigs;
import com.ctre.phoenix6.configs.SoftwareLimitSwitchConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.CANrange;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.wpilibj.DriverStation;

import java.util.function.Supplier;

/** Add your docs here. */
public class PhoneixUtil {
	public static boolean checkErrorAndRetry(Supplier<StatusCode> function, int numTries) {
		StatusCode code = function.get();
		int tries = 0;
		while (code != StatusCode.OK && tries < numTries) {
			DriverStation.reportWarning("Retrying CTRE Device Config " + code.getName(), false);
			code = function.get();
			tries++;
		}
		if (code != StatusCode.OK) {
			DriverStation.reportError("Failed to execute phoenix pro api call after " + numTries + " attempts", false);
			return false;
		}
		return true;
	}

	public static boolean checkErrorAndRetry(Supplier<StatusCode> function) {
		return checkErrorAndRetry(function, 5);
	}

	public static boolean applyConfiguration(TalonFX motor, TalonFXConfiguration config) {
		return checkErrorAndRetry(() -> motor.getConfigurator().apply(config), motor.getDeviceID());
	}

	public static boolean applyConfiguration(TalonFX motor, SoftwareLimitSwitchConfigs config) {
		return checkErrorAndRetry(() -> motor.getConfigurator().apply(config));
	}

	public static boolean applyConfiguration(TalonFX motor, HardwareLimitSwitchConfigs config) {
		return checkErrorAndRetry(() -> motor.getConfigurator().apply(config));
	}

	public static boolean applyConfiguration(CANrange canrange, CANrangeConfiguration config) {
		return checkErrorAndRetry(() -> canrange.getConfigurator().apply(config), canrange.getDeviceID());
	}

}
