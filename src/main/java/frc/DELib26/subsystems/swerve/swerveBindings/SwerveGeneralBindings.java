package frc.DELib26.subsystems.swerve.swerveBindings;

import frc.DELib26.bindings.Bindings;
import frc.DELib26.subsystems.swerve.SwerveSubsystem;
import frc.DELib26.subsystems.swerve.swerveCommands.tuning.OdometryTest;
import frc.DELib26.subsystems.swerve.swerveCommands.tuning.SwervePush;
import frc.DELib26.util.RobotState;
import frc.DELib26.util.dashboard.DashboardUtils;

public class SwerveGeneralBindings extends Bindings {
	private final SwerveSubsystem swerve;
	private final RobotState robotState;

	private static final String DASHBOARD_FOLDER = "Swerve General";
	private static final String COMMANDS_FOLDER = DASHBOARD_FOLDER + "/Commands/";
	private static final String SWERVE_PUSH_KEY = COMMANDS_FOLDER + "Swerve Push";
	private static final String CMD_ODOMETRY_TEST = COMMANDS_FOLDER + "Odometry test no vision";

	public SwerveGeneralBindings(SwerveSubsystem swerve, RobotState robotState) {
		this.swerve = swerve;
		this.robotState = robotState;
	}

	@Override
	public void binding() {
		DashboardUtils.setData(SWERVE_PUSH_KEY, new SwervePush(swerve));
		DashboardUtils.setData(CMD_ODOMETRY_TEST, new OdometryTest(swerve, robotState));
	}

}
