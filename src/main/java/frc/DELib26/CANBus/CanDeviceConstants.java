package frc.DELib26.CANBus;

import com.ctre.phoenix6.CANBus;

public class CanDeviceConstants {
	public final int id;
	public final String bus;

	public CanDeviceConstants(int id, String bus) {
		this.id = id;
		this.bus = bus;
	}

	public CANBus getBus() {
		return CANBusRegistry.get(this.bus);
	}

}
