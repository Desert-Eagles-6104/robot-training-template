package frc.DELib26.subsystems.motorSubsystems.currentTripDetectors;

import frc.DELib26.util.TimestampDeltaCalculator;

// Spike trip: |dI/dt| >= threshold  &&  filtered >= guard
public final class SpikeCurrentDetector extends CurrentTripDetector {

	public static class SpikeCurrentDetectorConfig extends CurrentTripDetectorConfig {
		/**
		 * Minimum rate of change of current (dI/dt) for a spike to be considered a trip, in amps per second. Must be > 0.
		 */
		public double spikeThresholdAmpsPerSec;

		/**
		 * Minimum filtered current magnitude for a spike to be considered a trip. This prevents small noise from causing trips when the current is low.
		 */
		public double minMagnitudeGuardAmps;
	}

	private final double spikeThresholdAmpsPerSec, minMagnitudeGuardAmps;
	private final TimestampDeltaCalculator deltaCalculator;

	public SpikeCurrentDetector(SpikeCurrentDetectorConfig config) {
		super(config);
		if (config.spikeThresholdAmpsPerSec <= 0) throw new IllegalArgumentException("spikeThresholdAmpsPerSec must be > 0");
		if (config.minMagnitudeGuardAmps < 0) throw new IllegalArgumentException("minMagnitudeGuardAmps must be >= 0");
		this.deltaCalculator = new TimestampDeltaCalculator();
		this.spikeThresholdAmpsPerSec = config.spikeThresholdAmpsPerSec;
		this.minMagnitudeGuardAmps = config.minMagnitudeGuardAmps;
	}

	@Override
	public void initialize() {
		super.initialize();
		this.deltaCalculator.reset();
	}

	@Override
	protected boolean testCondition(double filtered, double prev) {
		double dIdt = (filtered - prev) / deltaCalculator.update();
		return Math.abs(dIdt) >= spikeThresholdAmpsPerSec && filtered >= minMagnitudeGuardAmps;
	}
}