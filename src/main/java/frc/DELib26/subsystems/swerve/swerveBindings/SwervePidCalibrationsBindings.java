package frc.DELib26.subsystems.swerve.swerveBindings;

import com.ctre.phoenix6.configs.Slot0Configs;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.DELib26.bindings.Bindings;
import frc.DELib26.subsystems.swerve.SwerveSubsystem;
import frc.DELib26.subsystems.swerve.swerveCommands.DriveToPoint;
import frc.DELib26.subsystems.swerve.swerveCommands.FaceHeading;
import frc.DELib26.subsystems.swerve.swerveCommands.TeleopDrive;
import frc.DELib26.subsystems.swerve.swerveCommands.tuning.SetDrivePidfSlotConfigs;
import frc.DELib26.subsystems.swerve.swerveCommands.tuning.SetRotationPidConfigs;
import frc.DELib26.subsystems.swerve.swerveCommands.tuning.SwerveSysID;
import frc.DELib26.subsystems.swerve.swerveUtil.SwerveConstants;
import frc.DELib26.subsystems.swerve.swerveUtil.SysIdMechanism;
import frc.DELib26.util.dashboard.DashboardUtils;

public class SwervePidCalibrationsBindings extends Bindings {

	private final SwerveSubsystem swerve;
	private final SwerveConstants swerveConstants;

	private static final String DASHBOARD_FOLDER = "Swerve Pid Calibrations";
	private static final String VALUES_FOLDER = DASHBOARD_FOLDER + "/Values/";

	private static final String DRIVE_PID_VALUES_FOLDER = VALUES_FOLDER + "Drive/";
	private static final String DRIVE_PID_KP_KEY = DRIVE_PID_VALUES_FOLDER + "kP";
	private static final String DRIVE_PID_KI_KEY = DRIVE_PID_VALUES_FOLDER + "kI";
	private static final String DRIVE_PID_KD_KEY = DRIVE_PID_VALUES_FOLDER + "kD";
	private static final String DRIVE_PID_KG_KEY = DRIVE_PID_VALUES_FOLDER + "kG";
	private static final String DRIVE_PID_KS_KEY = DRIVE_PID_VALUES_FOLDER + "kS";
	private static final String DRIVE_PID_KV_KEY = DRIVE_PID_VALUES_FOLDER + "kV";
	private static final String DRIVE_PID_KA_KEY = DRIVE_PID_VALUES_FOLDER + "kA";

	private static final String STEER_PID_VALUES_FOLDER = VALUES_FOLDER + "Steer/";
	private static final String STEER_PID_KP_KEY = STEER_PID_VALUES_FOLDER + "kP";
	private static final String STEER_PID_KI_KEY = STEER_PID_VALUES_FOLDER + "kI";
	private static final String STEER_PID_KD_KEY = STEER_PID_VALUES_FOLDER + "kD";
	private static final String STEER_PID_KG_KEY = STEER_PID_VALUES_FOLDER + "kG";
	private static final String STEER_PID_KS_KEY = STEER_PID_VALUES_FOLDER + "kS";
	private static final String STEER_PID_KV_KEY = STEER_PID_VALUES_FOLDER + "kV";
	private static final String STEER_PID_KA_KEY = STEER_PID_VALUES_FOLDER + "kA";

	private static final String ROTATION_PID_VALUES_FOLDER = VALUES_FOLDER + "Rotation/";
	public static final String ROTATION_PID_KP_KEY = ROTATION_PID_VALUES_FOLDER + "kP";
	public static final String ROTATION_PID_KI_KEY = ROTATION_PID_VALUES_FOLDER + "kI";
	public static final String ROTATION_PID_KD_KEY = ROTATION_PID_VALUES_FOLDER + "kD";

	private static final String COMMANDS_FOLDER = DASHBOARD_FOLDER + "/Commands/";

	private static final String CMD_FACE_0_KEY = COMMANDS_FOLDER + "Face 0";
	private static final String CMD_FACE_90_KEY = COMMANDS_FOLDER + "Face 90";
	private static final String CMD_GO_PLUS_1M_X_KEY = COMMANDS_FOLDER + "Go +1m X";
	private static final String CMD_GO_MINUS_1M_X_KEY = COMMANDS_FOLDER + "Go -1m X";
	private static final String CMD_TELEOP_DRIVE_KEY = COMMANDS_FOLDER + "Teleop drive";

	private static final String CMD_TRANSLATION_SYS_ID_KEY = COMMANDS_FOLDER + "Run translation sysId";
	private static final String CMD_ROTATION_SYS_ID_KEY = COMMANDS_FOLDER + "Run rotation sysId";
	private static final String CMD_STEER_SYS_ID_KEY = COMMANDS_FOLDER + "Run steer sysId";

	private Slot0Configs initDriveSlot0;
	private Slot0Configs initSteerSlot0;

	public SwervePidCalibrationsBindings(SwerveSubsystem swerve, SwerveConstants swerveConstants) {
		this.swerve = swerve;
		this.swerveConstants = swerveConstants;
	}

	@Override
	protected void binding() {

		DashboardUtils.setData(CMD_FACE_0_KEY,
			new FaceHeading(this.swerve,
				() -> Rotation2d.fromDegrees(0)
			)
		);

		DashboardUtils.setData(CMD_FACE_90_KEY,
			new FaceHeading(this.swerve,
				() -> Rotation2d.fromDegrees(90)
			)
		);

		DashboardUtils.setData(CMD_GO_PLUS_1M_X_KEY,
			new DriveToPoint(
				this.swerve,
				() -> this.getRelativePose(new Pose2d(1, 0, Rotation2d.fromDegrees(0)))
			)
		);

		DashboardUtils.setData(CMD_GO_MINUS_1M_X_KEY,
			new DriveToPoint(
				this.swerve,
				() -> this.getRelativePose(new Pose2d(-1, 0, Rotation2d.fromDegrees(0)))
			)
		);

		DashboardUtils.setData(CMD_TELEOP_DRIVE_KEY, new TeleopDrive(this.swerve));

		this.initDriveSlot0 = this.swerveConstants.DRIVE_SLOT0_CONFIGS;
		this.initSteerSlot0 = this.swerveConstants.STEER_SLOT0_CONFIGS;

		Command setDrivePidfCmd = SetDrivePidfSlotConfigs.slot0(
			swerve,
			() -> new Slot0Configs()
					.withKP(SmartDashboard.getNumber(DRIVE_PID_KP_KEY, this.initDriveSlot0.kP))
					.withKI(SmartDashboard.getNumber(DRIVE_PID_KI_KEY, this.initDriveSlot0.kI))
					.withKD(SmartDashboard.getNumber(DRIVE_PID_KD_KEY, this.initDriveSlot0.kD))
					.withKG(SmartDashboard.getNumber(DRIVE_PID_KG_KEY, this.initDriveSlot0.kG))
					.withKS(SmartDashboard.getNumber(DRIVE_PID_KS_KEY, this.initDriveSlot0.kS))
					.withKV(SmartDashboard.getNumber(DRIVE_PID_KV_KEY, this.initDriveSlot0.kV))
					.withKA(SmartDashboard.getNumber(DRIVE_PID_KA_KEY, this.initDriveSlot0.kA))
		);

		Command setSteerPidfCmd = SetDrivePidfSlotConfigs.slot0(
			swerve,
			() -> new Slot0Configs()
					.withKP(SmartDashboard.getNumber(STEER_PID_KP_KEY, this.initSteerSlot0.kP))
					.withKI(SmartDashboard.getNumber(STEER_PID_KI_KEY, this.initSteerSlot0.kI))
					.withKD(SmartDashboard.getNumber(STEER_PID_KD_KEY, this.initSteerSlot0.kD))
					.withKG(SmartDashboard.getNumber(STEER_PID_KG_KEY, this.initSteerSlot0.kG))
					.withKS(SmartDashboard.getNumber(STEER_PID_KS_KEY, this.initSteerSlot0.kS))
					.withKV(SmartDashboard.getNumber(STEER_PID_KV_KEY, this.initSteerSlot0.kV))
					.withKA(SmartDashboard.getNumber(STEER_PID_KA_KEY, this.initSteerSlot0.kA))
		);

		Command setRotationPidCmd = new SetRotationPidConfigs(
			swerve,
			() -> SmartDashboard.getNumber(ROTATION_PID_KP_KEY, swerve.getHeadingController().getP()),
			() -> SmartDashboard.getNumber(ROTATION_PID_KI_KEY, swerve.getHeadingController().getI()),
			() -> SmartDashboard.getNumber(ROTATION_PID_KD_KEY, swerve.getHeadingController().getD())
		);

		DashboardUtils.registerTriggerOnChange(DRIVE_PID_KP_KEY, this.initDriveSlot0.kP)
				.onTrue(setDrivePidfCmd);
		DashboardUtils.registerTriggerOnChange(DRIVE_PID_KI_KEY, this.initDriveSlot0.kI)
				.onTrue(setDrivePidfCmd);
		DashboardUtils.registerTriggerOnChange(DRIVE_PID_KD_KEY, this.initDriveSlot0.kD)
				.onTrue(setDrivePidfCmd);
		DashboardUtils.registerTriggerOnChange(DRIVE_PID_KG_KEY, this.initDriveSlot0.kG)
				.onTrue(setDrivePidfCmd);
		DashboardUtils.registerTriggerOnChange(DRIVE_PID_KS_KEY, this.initDriveSlot0.kS)
				.onTrue(setDrivePidfCmd);
		DashboardUtils.registerTriggerOnChange(DRIVE_PID_KV_KEY, this.initDriveSlot0.kV)
				.onTrue(setDrivePidfCmd);
		DashboardUtils.registerTriggerOnChange(DRIVE_PID_KA_KEY, this.initDriveSlot0.kA)
				.onTrue(setDrivePidfCmd);

		DashboardUtils.registerTriggerOnChange(STEER_PID_KP_KEY, this.initSteerSlot0.kP)
				.onTrue(setSteerPidfCmd);
		DashboardUtils.registerTriggerOnChange(STEER_PID_KI_KEY, this.initSteerSlot0.kI)
				.onTrue(setSteerPidfCmd);
		DashboardUtils.registerTriggerOnChange(STEER_PID_KD_KEY, this.initSteerSlot0.kD)
				.onTrue(setSteerPidfCmd);
		DashboardUtils.registerTriggerOnChange(STEER_PID_KG_KEY, this.initSteerSlot0.kG)
				.onTrue(setSteerPidfCmd);
		DashboardUtils.registerTriggerOnChange(STEER_PID_KS_KEY, this.initSteerSlot0.kS)
				.onTrue(setSteerPidfCmd);
		DashboardUtils.registerTriggerOnChange(STEER_PID_KV_KEY, this.initSteerSlot0.kV)
				.onTrue(setSteerPidfCmd);
		DashboardUtils.registerTriggerOnChange(STEER_PID_KA_KEY, this.initSteerSlot0.kA)
				.onTrue(setSteerPidfCmd);

		DashboardUtils.registerTriggerOnChange(ROTATION_PID_KP_KEY, swerve.getHeadingController().getP())
				.onTrue(setRotationPidCmd);
		DashboardUtils.registerTriggerOnChange(ROTATION_PID_KI_KEY, swerve.getHeadingController().getI())
				.onTrue(setRotationPidCmd);
		DashboardUtils.registerTriggerOnChange(ROTATION_PID_KD_KEY, swerve.getHeadingController().getD())
				.onTrue(setRotationPidCmd);


		DashboardUtils.setData(CMD_ROTATION_SYS_ID_KEY,
			new SwerveSysID(swerve, swerveConstants, SysIdMechanism.SWERVE_ROTATION, true)
		);
		DashboardUtils.setData(CMD_TRANSLATION_SYS_ID_KEY,
			new SwerveSysID(swerve, swerveConstants, SysIdMechanism.SWERVE_TRANSLATION, true)
		);
		DashboardUtils.setData(CMD_STEER_SYS_ID_KEY,
			new SwerveSysID(swerve, swerveConstants, SysIdMechanism.SWERVE_STEER, true)
		);


	}

	private Pose2d getRelativePose(Pose2d offset) {
		Pose2d current = this.swerve.getPose();
		return new Pose2d(
				current.getTranslation().plus(offset.getTranslation().rotateBy(current.getRotation())),
				current.getRotation().plus(offset.getRotation()));
	}
}
