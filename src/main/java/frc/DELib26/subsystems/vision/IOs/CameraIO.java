package frc.DELib26.subsystems.vision.IOs;

import frc.DELib26.subsystems.vision.visionUtil.MegatagPoseEstimate;

public interface CameraIO {

	public class CameraIOInputs {
		public MegatagPoseEstimate megatag1PoseEstimate;
		public MegatagPoseEstimate megatag2PoseEstimate;
		public boolean seesTarget;

		// [MT1x, MT1y, MT1z, MT1roll, MT1pitch, MT1Yaw, MT2x, MT2y, MT2z, MT2roll, MT2pitch, MT2yaw]
		//public double[] standardDeviations = new double[12];

		public CameraIOInputs copy() {
			CameraIOInputs c = new CameraIOInputs();
			c.seesTarget = this.seesTarget;
			c.megatag1PoseEstimate = (this.megatag1PoseEstimate != null) ? this.megatag1PoseEstimate : null;
			c.megatag2PoseEstimate = (this.megatag2PoseEstimate != null) ? this.megatag2PoseEstimate : null;
			return c;
		}

	}

	public void refreshData();
	public void updateInputs(CameraIOInputs inputs);
	public void setThrottle(int throttle);

}
