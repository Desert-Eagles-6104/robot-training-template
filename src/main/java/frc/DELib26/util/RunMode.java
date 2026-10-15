package frc.DELib26.util;

import edu.wpi.first.wpilibj.RobotBase;
import org.littletonrobotics.junction.Logger;

public enum RunMode {
	/**
	 * Running on a real robot.
	 */
	REAL,

	/**
	 * Running a physics simulator.
	 */
	SIM,

	/**
	 * Replaying from a log file.
	 */
	REPLAY;

	private static volatile RunMode CURRENT = detect();

	public static boolean isReal() { return CURRENT == REAL; }
	public static boolean isSim() { return CURRENT == SIM; }
	public static boolean isReplay() { return CURRENT == REPLAY; }

	/** Returns the cached current run mode. */
	public static RunMode current() {
		return CURRENT;
	}

	private static RunMode detect() {
		if (Logger.hasReplaySource()) return RunMode.REPLAY;
		return RobotBase.isReal() ? RunMode.REAL : RunMode.SIM;
	}
}
