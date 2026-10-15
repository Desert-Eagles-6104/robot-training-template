package frc.DELib26.subsystems.motorSubsystems.commands;

import edu.wpi.first.wpilibj.Timer;
import frc.DELib26.subsystems.motorSubsystems.motorBase.MotorSubsystemTalon;

import java.util.function.DoubleSupplier;

public abstract class MotorSubsystemHoming<T extends MotorSubsystemTalon> extends MotorSubsystemSetPercentage<T> {
	protected final Timer dwellTimer = new Timer();
	protected final double dwellTimeSeconds;

	private final double timeoutSeconds;
	private final Timer overallTimer = new Timer();

	public MotorSubsystemHoming(T motorSubsystem, DoubleSupplier percent, double dwellTimeSeconds) {
		this(motorSubsystem, percent, dwellTimeSeconds, -1);
	}

	/**
	 * @param motorSubsystem the motor to home
	 * @param percent open-loop output during homing (-1..1)
	 * @param dwellTimeSeconds time that homing condition must be met before homing is successful
	 * @param timeoutSeconds optional max duration; <=0 disables timeout
	 */
	public MotorSubsystemHoming(
		T motorSubsystem,
		DoubleSupplier percent,
		double dwellTimeSeconds,
		double timeoutSeconds
	) {
		super(motorSubsystem, percent, true);
		this.dwellTimeSeconds = dwellTimeSeconds;
		this.timeoutSeconds = timeoutSeconds;
	}

	@Override
	public void initialize() {
		super.initialize();
		this.dwellTimer.stop();
		this.dwellTimer.reset();
		this.overallTimer.restart();
	}

	@Override
	public void execute() {
		if (this.dwellTimer.isRunning()) {
			// Once we’ve started the dwell period, stop driving the motor
			this.motorSubsystem.disableMotors();
		} else {
			super.execute();
		}
	}

	@Override
	public void end(boolean interrupted) {
		if (!interrupted) {
			this.motorSubsystem.resetPosition(motorSubsystem.configuration.homePositionUnits);
		}
		super.end(interrupted);
	}

	protected abstract boolean isHomeConditionMet();

	@Override
	public boolean isFinished() {
		if (timeoutSeconds > 0 && overallTimer.hasElapsed(timeoutSeconds)) {
			return true;
		}

		if (isHomeConditionMet()) {
			if (!dwellTimer.isRunning()) {
				dwellTimer.restart();
			}
		} else {
			if (dwellTimer.isRunning()) {
				dwellTimer.stop();
				dwellTimer.reset();
			}
		}

		return dwellTimer.isRunning() && dwellTimer.hasElapsed(dwellTimeSeconds);
	}
}
