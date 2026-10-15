package frc.DELib26.util.vision;

/**
 * Camera models for various Limelight versions.
 */
public enum CameraModel {
	/** Limelight 2 / 2+: 62.5° x 48.9°. Max pipeline res commonly 960x720. */
	LIMELIGHT_2(62.5, 48.9, 960, 720), // :contentReference[oaicite:1]{index=1}

	/** Limelight 3 (color, rolling shutter): same FOV as LL2. Max sensor mode: 2592x1944 (5MP). */
	LIMELIGHT_3(62.5, 48.9, 2592, 1944), // :contentReference[oaicite:2]{index=2}

	/** Limelight 3A (color, rolling shutter): same FOV as LL2. Max capture mode: 1280x960. */
	LIMELIGHT_3A(62.5, 48.9, 1280, 960), // :contentReference[oaicite:3]{index=3}

	/** Limelight 3G (mono, global shutter): wider ~80-82° x 56.2°. Max capture mode: 1280x800. */
	LIMELIGHT_3G(82.0, 56.2, 1280, 800), // :contentReference[oaicite:4]{index=4}

	/** Limelight 4: same lens FOV as 3G (82° x 56.2°). OV9281 mode: 1280x800. */
	LIMELIGHT_4(82.0, 56.2, 1280, 800); // :contentReference[oaicite:5]{index=5}

	public final double horizontalFOVDegrees;
	public final double verticalFOVDegrees;

	/** Maximum supported capture/pipeline resolution (pixels). */
	public final int maxWidthPx;
	public final int maxHeightPx;

	CameraModel(double horizontalFOVDegrees, double verticalFOVDegrees, int maxWidthPx, int maxHeightPx) {
		this.horizontalFOVDegrees = horizontalFOVDegrees;
		this.verticalFOVDegrees = verticalFOVDegrees;
		this.maxWidthPx = maxWidthPx;
		this.maxHeightPx = maxHeightPx;
	}

	public double maxAspectRatio() {
		return (double) maxWidthPx / (double) maxHeightPx;
	}
}