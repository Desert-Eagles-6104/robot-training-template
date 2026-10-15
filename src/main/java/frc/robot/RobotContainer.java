package frc.robot;

import com.ctre.phoenix6.CANBus;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.DELib26.CANBus.CANBusRegistry;
import frc.DELib26.CANBus.CANBusStatusLogger;
import frc.DELib26.autos.PathPlannerAutoBuilder;
import frc.DELib26.controller.ControllerType;
import frc.DELib26.controller.GenericController;
import frc.DELib26.subsystems.swerve.SwerveIO;
import frc.DELib26.subsystems.swerve.SwerveIOHardware;
import frc.DELib26.subsystems.swerve.SwerveIOSim;
import frc.DELib26.subsystems.swerve.SwerveSubsystem;
import frc.DELib26.subsystems.swerve.swerveCommands.TeleopDrive;
import frc.DELib26.subsystems.vision.VisionSubsystem;
import frc.DELib26.util.RunMode;
import frc.robot.constants.Constants;
import frc.robot.sim.FieldVisualization;
import frc.robot.sim.SimCoordinator;
import frc.robot.subsystems.vision.Vision2026;

import java.util.Optional;

public class RobotContainer {
	private static RobotContainer instance;

	public static RobotContainer getInstance() {
		if (instance == null) {
			instance = new RobotContainer();
		}
		return instance;
	}

	// logging
	private static enum LoggingMode {
		NONE,
		BASIC,
		EXTENDED
	}

	private final LoggingMode loggingMode;
	// context
	public final RobotModel robotModel = RobotModel.ROBOT_2026;
	public final RobotState2026 robotState2026;
	public final Optional<SimCoordinator> simCoordinator;
	// controllers
	// We use ps5 as default because that is what we use in the competition
	public final GenericController driverController = ControllerType.PS5.buildController(0);
	// subsystems
	public final Constants constants;
	public final SwerveSubsystem swerveSubsystem;
	public final VisionSubsystem visionSubsystem;

	// autos
	public final PathPlannerAutoBuilder autoBuilder;

	@SuppressWarnings("DuplicateBranches")
	private RobotContainer() {
		final boolean isSim = RunMode.isSim();

		this.loggingMode = isSim ? LoggingMode.EXTENDED : LoggingMode.EXTENDED;

		this.constants = this.robotModel.getConstants();

		this.robotState2026 = new RobotState2026(this.constants.visionConfig);

		final SwerveIO swerveIO = isSim
				? new SwerveIOSim(this.constants.swerveConstants)
				: new SwerveIOHardware(this.constants.swerveConstants);

		this.swerveSubsystem = new SwerveSubsystem(
				swerveIO,
				this.driverController,
				this.constants.swerveConstants,
				robotState2026);

		this.visionSubsystem = new Vision2026(this.constants.visionConfig, this.robotState2026);

		if (isSim) {
			this.simCoordinator =  Optional.of(new SimCoordinator(
					this.robotState2026,
					this.constants.simulationConstants.get(),
					((SwerveIOSim) swerveIO).mapleSimSwerveDrivetrain.mapleSimDrive
				));
		} else {
			this.simCoordinator = Optional.empty();
		}

		this.autoBuilder = new PathPlannerAutoBuilder(this.swerveSubsystem, this.constants);
	}

	public void configureTeleopBindings() {
		// we dont need to care about Disconnects or controls or switching because on
		// any disconnect it will automatically disable.

		this.driverController.assignControllerType();

		this.swerveSubsystem.setDefaultCommand(new TeleopDrive(this.swerveSubsystem));

		// pose reset
		this.driverController.options()
				.onTrue(new InstantCommand(() -> this.swerveSubsystem.resetRotationToAllianceForward()));
	}

	public final void logTelemetry() {
		if (this.loggingMode != LoggingMode.NONE) {

			if (RunMode.isSim()) {
				FieldVisualization.getInstance().updateViz();
			} else {
				// CAN bus logging does not work in sim
				for (CANBus bus : CANBusRegistry.getAllCANBuses()) {
					CANBusStatusLogger.logStatus(bus);
				}
			}

		}

		if (this.loggingMode == LoggingMode.EXTENDED) {
			this.swerveSubsystem.logExtendedTelemetry();
			this.visionSubsystem.logExtendedTelemetry();
		} else if (this.loggingMode == LoggingMode.BASIC) {
			this.swerveSubsystem.logTelemetry();
			this.visionSubsystem.logTelemetry();
		}
	}

	public final void disableAll() {
		this.swerveSubsystem.stop();
	}

	public Command getAutonomousCommand() {
		return BindingsManager.getInstance().autoBindings.getAuto().commandFactory().get();
	}

}
