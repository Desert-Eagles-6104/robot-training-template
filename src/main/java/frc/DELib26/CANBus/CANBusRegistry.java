package frc.DELib26.CANBus;

import com.ctre.phoenix6.CANBus;

import java.util.Collection;
import java.util.concurrent.ConcurrentHashMap;
import java.util.concurrent.ConcurrentMap;

public final class CANBusRegistry {
	private static final ConcurrentMap<String, CANBus> cache = new ConcurrentHashMap<>();

	private CANBusRegistry() {}

	/** Returns the same CANBus instance for the same name every time. */
	public static CANBus get(String name) {
		String key = (name == null) ? "" : name;
		return cache.computeIfAbsent(key, k -> new CANBus(k));
	}

	public static Collection<CANBus> getAllCANBuses() {
		return cache.values();
	}
}