package frc.robot.constants;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.NeutralModeValue;
import frc.DELib26.subsystems.swerve.swerveUtil.SwerveConstants;
import frc.DELib26.subsystems.vision.visionUtil.VisionSubsystemConfiguration;
import frc.DELib26.util.vision.CameraConfiguration;
import frc.DELib26.util.vision.CameraModel;

import java.util.Optional;

public class Robot2026Constants extends Constants {

	public Robot2026Constants() {
		super.swerveConstants = new Robot2026SwerveConstants();
		super.visionConfig = new Robot2026VisionConfig();
		super.autonomousConstants = new Robot2026AutonomousConstants();

		super.simulationConstants = Optional.of(new Robot2026SimulationConstants());
	}

	private static class Robot2026SwerveConstants extends SwerveConstants {
		public Robot2026SwerveConstants() {
		}

		@Override
		protected TalonFXConfiguration getDefaultDriveTalonFXConfig() {
			return new TalonFXConfiguration()
					.withCurrentLimits(new CurrentLimitsConfigs()
							.withSupplyCurrentLimitEnable(true)
							.withStatorCurrentLimitEnable(true)
							.withSupplyCurrentLimit(30.0)
							.withStatorCurrentLimit(DRIVE_STATOR_CURRENT_LIMIT))
					.withMotorOutput(new MotorOutputConfigs()
							.withNeutralMode(NeutralModeValue.Brake));
		}

		@Override
		protected TalonFXConfiguration getDefaultSteerTalonFXConfig() {
			return new TalonFXConfiguration()
					.withCurrentLimits(new CurrentLimitsConfigs()
							.withSupplyCurrentLimitEnable(true)
							.withStatorCurrentLimitEnable(true)
							.withSupplyCurrentLimit(30.0)
							.withStatorCurrentLimit(STEER_STATOR_CURRENT_LIMIT))
					.withMotorOutput(new MotorOutputConfigs()
							.withNeutralMode(NeutralModeValue.Brake));
		}
	}

	private static class Robot2026VisionConfig extends VisionSubsystemConfiguration {
		public Robot2026VisionConfig() {
			super.cameraConfigurations = new CameraConfiguration[] {
					new CameraConfiguration(
							"limelight-shooter",
							CameraModel.LIMELIGHT_4,
							-0.28493,
							-0.27445,
							0.53572,
							0.0,
							17.96,
							270.0),
			};
			this.filteredAprilTagDataForGyro = Optional.empty();// Optional.of(FieldConstants.APRIL_TAG_LAYOUT_REEFS_ONLY);
			this.aprilTagFieldLayout = FieldConstants.FIELD_LAYOUT;
		}
	}

	private static class Robot2026AutonomousConstants extends AutonomousConstants {
		public Robot2026AutonomousConstants() {
			MAX_START_TRANS_ERROR_METERS = 0.8;
			MAX_START_HEADING_ERROR_DEGREES = 180;
			AUTONOMOUS_VISION_LOOKBACK_TIME_SECONDS = 0.5;
		}
	}

	private static class Robot2026SimulationConstants extends SimulationConstants {
		public Robot2026SimulationConstants() {}
	}

}
