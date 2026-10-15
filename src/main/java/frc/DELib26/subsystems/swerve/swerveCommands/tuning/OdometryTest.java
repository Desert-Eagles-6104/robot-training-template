package frc.DELib26.subsystems.swerve.swerveCommands.tuning;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.DELib26.subsystems.swerve.SwerveSubsystem;
import frc.DELib26.util.RobotState;

public class OdometryTest extends Command {
	private final SwerveSubsystem swerve;
	private final RobotState robotState;
	private Pose2d startingPose = new Pose2d();

	public OdometryTest(SwerveSubsystem swerve, RobotState robotState) {
		this.swerve = swerve;
		this.robotState = robotState;
	}

	@Override
	public void initialize() {
		robotState.setUseVision(false);
		this.startingPose = swerve.getPose();
	}

	@Override
	public void end(boolean interrupted) {
		this.robotState.setUseVision(true);
		Pose2d current = swerve.getPose();

		double offset = current.getTranslation().getDistance(startingPose.getTranslation());
		System.out.println("Odometry Test Result: " + offset + " meters");
		System.out.println("X offset: " + (current.getX() - startingPose.getX()) + " meters");
		System.out.println("Y offset: " + (current.getY() - startingPose.getY()) + " meters");
	}
}
