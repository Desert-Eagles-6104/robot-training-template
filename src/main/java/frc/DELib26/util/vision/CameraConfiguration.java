package frc.DELib26.util.vision;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;

public class CameraConfiguration {
	public String limelightTableName;
	public double forward;
	public double side;
	/**
	 * Height of the camera off the ground in meters.
	 */
	public double up;
	/**
	 * Roll of the camera relative to the robot's forward direction in degrees.
	 */
	public double roll;
	/**
	 * Pitch of the camera relative to the robot's forward direction in degrees.
	 * (for example, a camera angled parallel to the ground would have a pitch of 0
	 * degrees)
	 */
	public double pitch;
	/**
	 * Yaw of the camera relative to the robot's forward direction in degrees.
	 */
	public double yaw;

	public final CameraModel cameraModel;

	public CameraConfiguration(
			String limelightTableName,
			CameraModel cameraModel,
			double forward,
			double side,
			double up,
			double roll,
			double pitch,
			double yaw) {
		this.limelightTableName = limelightTableName;
		this.cameraModel = cameraModel;
		this.forward = forward;
		this.side = side;
		this.up = up;
		this.roll = roll;
		this.pitch = pitch;
		this.yaw = yaw;
	}

	public String getName() {
		return limelightTableName.replace("limelight-", "");
	}

	public Transform3d getRobotToCamera() {
		return new Transform3d(
				new Translation3d(
						this.forward,
						this.side,
						this.up),
				new Rotation3d(
						Units.degreesToRadians(this.roll),
						-Units.degreesToRadians(this.pitch),
						Units.degreesToRadians(this.yaw)));
	}

	public Transform2d getRobotToCamera2d() {
		return new Transform2d(
				new Translation2d(this.forward, this.side),
				Rotation2d.fromDegrees(yaw));
	}
}
