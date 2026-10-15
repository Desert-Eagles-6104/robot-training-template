package frc.DELib26.CANBus.CANrange;

import com.ctre.phoenix6.configs.CANrangeConfiguration;
import com.ctre.phoenix6.hardware.CANrange;
import frc.DELib26.CANBus.CanDeviceConstants;
import frc.DELib26.util.PhoneixUtil;

public class CANrangeConstants extends CanDeviceConstants {
	private final CANrangeConfiguration config;

	public CANrangeConstants(int id, String bus) {
		this(id, bus, new CANrangeConfiguration());
	}

	public CANrangeConstants(int id, String bus, CANrangeConfiguration config) {
		super(id, bus);
		this.config = config;
	}

	public CANrangeConfiguration getConfig() {
		return this.config;
	}

	public static CANrange createCANrange(CANrangeConstants constants) {
		CANrange canrange = new CANrange(constants.id, constants.getBus());
		canrange.clearStickyFaults();
		PhoneixUtil.applyConfiguration(canrange, constants.getConfig());
		return canrange;
	}
}