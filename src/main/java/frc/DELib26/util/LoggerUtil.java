package frc.DELib26.util;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import org.littletonrobotics.junction.Logger;

import java.nio.file.Files;
import java.nio.file.Path;
import java.nio.file.Paths;

public class LoggerUtil {
	/**
	 * A constant representing the absence of a valid double value.
	 * NaN value causes the pose to be ignored or undrawn in AdvantageScope.
	 * Uses {@link Double#NaN}.
	 */
	public static final double NO_DOUBLE = Double.NaN;

	/**
	 * A constant representing the absence of a valid 2D rotation.
	 * NaN rotation causes the pose to be ignored or undrawn in AdvantageScope.
	 * Uses {@link Rotation2d} initialized with {@link Double#NaN}.
	 */
	public static final Rotation2d NO_ROTATION_2D = new Rotation2d(Double.NaN);

	/**
	 * A constant representing the absence of a valid 3D rotation.
	 * NaN rotation causes the pose to be ignored or undrawn in AdvantageScope.
	 * Uses {@link Rotation3d} initialized with {@link Double#NaN}.
	 */
	public static final Rotation3d NO_ROTATION_3D = new Rotation3d(Double.NaN, Double.NaN, Double.NaN);

	/**
	 * A constant representing the absence of a valid 2D pose.
	 * NaN pose causes the pose to be ignored or undrawn in AdvantageScope.
	 * Uses {@link Pose2d} initialized with {@link Double#NaN} and {@link #NO_ROTATION_2D}.
	 */
	public static final Pose2d NO_POSE_2D = new Pose2d(Double.NaN, Double.NaN, NO_ROTATION_2D);

	/**
	 * A constant representing the absence of a valid 3D pose.
	 * NaN pose causes the pose to be ignored or undrawn in AdvantageScope.
	 * Uses {@link Pose3d} initialized with {@link Double#NaN} and {@link #NO_ROTATION_3D}.
	 */
	public static final Pose3d NO_POSE_3D = new Pose3d(Double.NaN, Double.NaN, Double.NaN, NO_ROTATION_3D);

	public static Path getDefaultLogDirectory() {
		if (RunMode.isReal()) {
			return getRealDefaultLogDirectory();
		} else {
			return getSimDefaultLogDirectory();
		}
	}

	public static Path getRealDefaultLogDirectory() {
		if (Files.isDirectory(Paths.get("/U"))) {
			return Paths.get("/U");
		} else {
			return Paths.get("/home/lvuser");
		}
	}

	public static Path getSimDefaultLogDirectory() {
		return getComputerDefaultLogDirectory().resolve("sim");
	}

	public static Path getComputerDefaultLogDirectory() {
		return Paths.get("").toAbsolutePath().resolve("wpilogs");
	}

	public static void recordOutput(String key, Double value) {
		Logger.recordOutput(key, value == null ? NO_DOUBLE : value);
	}

	public static void recordOutput(String key, Rotation2d value) {
		Logger.recordOutput(key, value == null ? NO_ROTATION_2D : value);
	}

	public static void recordOutput(String key, Rotation3d value) {
		Logger.recordOutput(key, value == null ? NO_ROTATION_3D : value);
	}

	public static void recordOutput(String key, Pose2d value) {
		Logger.recordOutput(key, value == null ? NO_POSE_2D : value);
	}

	public static void recordOutput(String key, Pose3d value) {
		Logger.recordOutput(key, value == null ? NO_POSE_3D : value);
	}

}
