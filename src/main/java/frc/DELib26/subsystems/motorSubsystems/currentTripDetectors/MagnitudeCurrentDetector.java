package frc.DELib26.subsystems.motorSubsystems.currentTripDetectors;

// Magnitude trip: filtered >= limit
public final class MagnitudeCurrentDetector extends CurrentTripDetector {

	public static class MagnitudeCurrentDetectorConfig extends CurrentTripDetectorConfig {
		/**
		 * current limit (amps)
		 */
		public double currentLimitAmps;
	}

	private final double currentLimitAmps;

	public MagnitudeCurrentDetector(MagnitudeCurrentDetectorConfig config) {
		super(config);
		if (config.currentLimitAmps <= 0) throw new IllegalArgumentException("currentLimitAmps must be > 0");
		this.currentLimitAmps = config.currentLimitAmps;
	}

	@Override
	protected boolean testCondition(double filtered, double prev) {
		return filtered >= currentLimitAmps;
	}
}