package frc.DELib26.CANBus;

import com.ctre.phoenix6.CANBus;
import org.littletonrobotics.junction.Logger;

/**
 * CAN Bus Status Logger Documentation:
 *
 * utilization:
 * <0.65: Good
 * 0.65-0.85: Getting congested
 * >0.85: Congested - Likely frame delays and stale device updates.
 * If you see spikes: You’re saturating the bus — too many devices or too-frequent status frames.
 * Fix: call optimizeBusUtilization() or reduce status frame rates (less telemetry).
 *
 * Transmit/Receive error counters:
 * - transmit_error_count (TEC): should be ≈0. Rising -> bad wiring, loose termination, or unplugged node.
 *   If it increases fast you'll hit "BusOff" soon — fix the physical layer.
 * - receive_error_count (REC): should be ≈0. Rising/spiking -> electrical noise or interference on CAN.
 *
 * Bus-off events:
 * Must remain 0 in matches. Each increment indicates a temporary total disconnect (~250 ms).
 *
 * TX queue full:
 * 0 is normal. Spikes mean software is flooding the bus (too many frames per loop).
 */
public class CANBusStatusLogger {

	public static void logStatus(CANBus bus) {
		var s = bus.getStatus();
		String base = "CANBusStatus/" + bus.getName();

		Logger.recordOutput(base + "/status", s.Status);
		/*
		 * <0.65: Good
		 * 0.65-0.85: Getting congested
		 * >0.85: Congested - Likely frame delays and stale device updates.
		 * If you see spikes: You’re saturating the bus — too many devices or too-frequent status frames.
		 * Fix: call optimizeBusUtilization() or reduce status frame rates (less telemetry).
		 */
		Logger.recordOutput(base + "/utilization", s.BusUtilization);
		/*
		 * Transmit/Receive error counters:
		 * - transmit_error_count (TEC): should be ≈0. Rising -> bad wiring, loose termination, or unplugged node.
		 *   If it increases fast you'll hit "BusOff" soon — fix the physical layer.
		 * - receive_error_count (REC): should be ≈0. Rising/spiking -> electrical noise or interference on CAN.
		 */
		Logger.recordOutput(base + "/transmit_error_count", s.TEC);
		Logger.recordOutput(base + "/receive_error_count",  s.REC);
		/*
		 * Bus-off events: must remain 0 in matches. Each increment indicates a temporary total disconnect (~250 ms).
		 */
		Logger.recordOutput(base + "/bus_off_count",  s.BusOffCount);
		/*
		 * TX queue full: 0 is normal. Spikes mean software is flooding the bus (too many frames per loop).
		 */
		Logger.recordOutput(base + "/tx_queue_full_count", s.TxFullCount);
	}
}