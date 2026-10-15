package frc.DELib26.subsystems.objectDetection.IOs;

import frc.DELib26.subsystems.objectDetection.util.ObjectDetectionFrame;
import frc.DELib26.util.SubsystemDataProcessor;
import frc.DELib26.util.vision.LimelightHelpers.RawDetection;

public interface ObjectDetectionIO extends SubsystemDataProcessor.IODataRefresher {

	public class ObjectDetectionIOInputs {
		public double lastUpdateTimestampSec;
		public RawDetection[] rawDetections = new RawDetection[0];
		public ObjectDetectionFrame latestFrame;

		public ObjectDetectionIOInputs copy() {
			ObjectDetectionIOInputs c = new ObjectDetectionIOInputs();
			c.lastUpdateTimestampSec = this.lastUpdateTimestampSec;
			c.rawDetections = this.rawDetections.clone();
			c.latestFrame = (this.latestFrame != null) ? this.latestFrame : null;
			return c;
		}
	}

	void updateInputs(ObjectDetectionIOInputs inputs);

}
