package frc.DELib26.util.dashboard;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.DELib26.booleanUtil.ChangeDetectorBoolean;

public class DashboardUtils {
	public static final String DEFAULT_DASHBOARD_TABLE = "SmartDashboard";

	public static Trigger registerTrigger(String key, boolean defaultValue) {
		SmartDashboard.setDefaultBoolean(key, defaultValue);
		return new Trigger(() -> SmartDashboard.getBoolean(key, defaultValue));
	}

	public static Trigger registerTriggerOnChange(String key, Double defaultValue) {
		SmartDashboard.setDefaultNumber(key, defaultValue);
		ChangeDetectorBoolean<Double> detector = new ChangeDetectorBoolean<Double>(defaultValue);
		return new Trigger(() -> detector.hasChanged(SmartDashboard.getNumber(key, defaultValue)));
	}

	public static void setData(String key, Command value) {
		SmartDashboard.putData(key, value);
	}

	public static <T> void setData(String key, SendableChooser<T> chooser, boolean isPersistent) {
		SmartDashboard.putData(key, chooser);
		if (isPersistent) {
			SmartDashboard.setPersistent(key);
		}
	}

	public static void setData(String key, double value, boolean isPersistent) {
		SmartDashboard.putNumber(key, value);
		if (isPersistent) {
			SmartDashboard.setPersistent(key);
		}
	}

	public static void setDefaultData(String key, double value, boolean isPersistent) {
		SmartDashboard.setDefaultNumber(key, value);
		if (isPersistent) {
			SmartDashboard.setPersistent(key);
		}
	}

	public static void setData(String key, double value[], boolean isPersistent) {
		SmartDashboard.putNumberArray(key, value);
		if (isPersistent) {
			SmartDashboard.setPersistent(key);
		}
	}

	public static void setDefaultData(String key, double value[], boolean isPersistent) {
		SmartDashboard.setDefaultNumberArray(key, value);
		if (isPersistent) {
			SmartDashboard.setPersistent(key);
		}
	}

	public static void setData(String key, boolean value, boolean isPersistent) {
		SmartDashboard.putBoolean(key, value);
		if (isPersistent) {
			SmartDashboard.setPersistent(key);
		}
	}

	public static void setDefaultData(String key, boolean value, boolean isPersistent) {
		SmartDashboard.setDefaultBoolean(key, value);
		if (isPersistent) {
			SmartDashboard.setPersistent(key);
		}
	}

	public static void setData(String key, String value, boolean isPersistent) {
		SmartDashboard.putString(key, value);
		if (isPersistent) {
			SmartDashboard.setPersistent(key);
		}
	}

	public static void setDefaultData(String key, String value, boolean isPersistent) {
		SmartDashboard.setDefaultString(key, value);
		if (isPersistent) {
			SmartDashboard.setPersistent(key);
		}
	}

}
