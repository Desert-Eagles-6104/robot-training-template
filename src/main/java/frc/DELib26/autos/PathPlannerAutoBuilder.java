package frc.DELib26.autos;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.config.ModuleConfig;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.pathfinding.Pathfinding;
import com.pathplanner.lib.util.PathPlannerLogging;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.DELib26.subsystems.swerve.SwerveSubsystem;
import frc.DELib26.subsystems.swerve.swerveUtil.SwerveConstants;
import frc.DELib26.util.BaseRobotConstants;
import frc.DELib26.util.FieldUtil;
import org.littletonrobotics.junction.Logger;

import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.function.Supplier;

/**
 * A utility class to build Path Planner autos.
 */
public class PathPlannerAutoBuilder {
	private RobotConfig robotConfig;
	private final Map<String, Supplier<PathPlannerAuto>> autosMap = new HashMap<>();

	public PathPlannerAutoBuilder(
			SwerveSubsystem swerveSubsystem, BaseRobotConstants constants, Map<String, Command> namedCommands) {
		SwerveConstants swerveConstants = constants.swerveConstants;
		Pathfinding.setPathfinder(new LocalADStarAK());

		this.robotConfig = getRobotConfig(constants);

		AutoBuilder.configure(
				swerveSubsystem::getPose,
				swerveSubsystem::resetPose,
				swerveSubsystem::getRobotSpeeds,
				(speeds, feedforwards) -> swerveSubsystem.pathPlannerDriveAuto(speeds, feedforwards),
				new PPHolonomicDriveController(
						swerveConstants.PATH_PLANNER_TRANSLATION_CONTROLLER.toPIDConstants(),
						swerveConstants.PATH_PLANNER_ROTATION_CONTROLLER.toPIDConstants()),
				this.robotConfig,
				() -> !FieldUtil.isBlueAllianceOrDefault(),
				swerveSubsystem
			);

		namedCommands.forEach(NamedCommands::registerCommand);

		List<String> pathNames = AutoBuilder.getAllAutoNames();
		pathNames.forEach(
				name -> this.autosMap.put(name, () -> (PathPlannerAuto) AutoBuilder.buildAuto(name))
			);

		// Logging
		PathPlannerLogging.setLogTargetPoseCallback(
				pose -> Logger.recordOutput("PathPlanner/TargetPose", pose));

		PathPlannerLogging.setLogActivePathCallback(
				poses -> Logger.recordOutput("PathPlanner/ActivePath", poses.toArray(Pose2d[]::new)));
	}

	public PathPlannerAutoBuilder(SwerveSubsystem swerveSubsystem, BaseRobotConstants constants) {
		this(swerveSubsystem, constants, new HashMap<String, Command>());
	}

	public PathConstraints getDefaultPathConstraints(SwerveConstants swerveConstants) {
		return new PathConstraints(
			swerveConstants.INITIAL_MAX_VELOCITY_FOR_DRIVE_TO_POINT,
			swerveConstants.MAX_LINEAR_ACCELERATION,
			swerveConstants.getMaxAngularSpeed(),
			swerveConstants.MAX_ANGULAR_ACCELERATION
		);
	}

	private static RobotConfig getRobotConfig(BaseRobotConstants constants) {
		ModuleConfig moduleCfg = new ModuleConfig(
				constants.swerveConstants.MODULE_HARDWARE_CONSTANTS.getWheelRadius(),
				// It wants the absolute max speed no reduction for friction,
				// it took me 7 hours to figure that out. fuck life.
				constants.swerveConstants.MODULE_HARDWARE_CONSTANTS.getTheoreticalMaxLinearSpeedMps(),
				constants.swerveConstants.WHEEL_COEFFICIENT_OF_FRICTION.in(Units.Value),
				constants.swerveConstants.MODULE_HARDWARE_CONSTANTS.driveMotor,
				constants.swerveConstants.MODULE_HARDWARE_CONSTANTS.driveGearRatio,
				constants.swerveConstants.DRIVE_STATOR_CURRENT_LIMIT.in(Units.Amps),
				1);

		RobotConfig robotConfig = new RobotConfig(
				constants.swerveConstants.ROBOT_MASS,
				constants.swerveConstants.ROBOT_MOMENT_OF_INERTIA,
				moduleCfg,
				constants.swerveConstants.getModuleLocations());

		return robotConfig;
	}

	public Map<String, Supplier<PathPlannerAuto>> getAutosMap() {
		return this.autosMap;
	}

	public static Command getResetOdomForPath(PathPlannerPath path) {
		// Get the starting pose from the path (blue-side origin)
		// At resetOdom it will fiip if needed
		Pose2d blueStart = path.getStartingHolonomicPose()
				.orElse(path.getStartingDifferentialPose());
		// Uses the resetPose consumer given in AutoBuilder.configure(...)
		return AutoBuilder.resetOdom(blueStart);
	}

	public static Command followPathWithReset(PathPlannerPath path) {
		return Commands.sequence(
				getResetOdomForPath(path),
				AutoBuilder.followPath(path)
		);
	}

	public static PathPlannerPath loadPath(String pathName) {
		try {
			return PathPlannerPath.fromPathFile(pathName);
		} catch (Exception e) {
			System.err.println("Error loading path: " + pathName);
			return null; // I dont care about null pointer exceptions here
		}
	}

	public static Command followNamedPath(String pathName) {
		return AutoBuilder.followPath(loadPath(pathName));
	}

	public static Command followPath(PathPlannerPath path) {
		return AutoBuilder.followPath(path);
	}

	public static Command addStopToPath(Command pathCommand, SwerveSubsystem swerveSubsystem) {
		return pathCommand.andThen(Commands.runOnce(() -> swerveSubsystem.stop() ));
	}

	public static Command followNamedPathWithStop(String pathName, SwerveSubsystem swerveSubsystem) {
		return addStopToPath(followNamedPath(pathName), swerveSubsystem);
	}

}
