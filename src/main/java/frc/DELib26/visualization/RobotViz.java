package frc.DELib26.visualization;

import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import frc.DELib26.util.ProjectConstants;
import frc.DELib26.util.TimestampDeltaCalculator;
import org.littletonrobotics.junction.mechanism.LoggedMechanism2d;

public class RobotViz {

	private static final double kCanvasWidthM = 1.3;
	private static final double kCanvasHeightM = 2;

	public static final Color8Bit WHITE = new Color8Bit(Color.kWhite);
	public static final Color8Bit RED = new Color8Bit(Color.kRed);
	public static final Color8Bit BLUE = new Color8Bit(Color.kBlue);
	public static final Color8Bit GREEN = new Color8Bit(Color.kGreen);

	protected final LoggedMechanism2d viz2d;

	protected final TimestampDeltaCalculator timestampDeltaCalculator;

	public RobotViz() {
		this.viz2d = new LoggedMechanism2d(kCanvasWidthM, kCanvasHeightM);
		this.timestampDeltaCalculator = new TimestampDeltaCalculator();
	}

	public static double computeFlywheelVizScale(double targetDegPerTickAtMax, double maxRpsExpected){
		return targetDegPerTickAtMax / (maxRpsExpected * ProjectConstants.DEFAULT_PERIOD_SECS * 360.0);
	}

	public static double computeExtenderLengthMeters(double ligament1widthMM, double ligament2widthMM){
		return 2 * (ligament1widthMM + ligament2widthMM) / 1000.0;
	}

}
