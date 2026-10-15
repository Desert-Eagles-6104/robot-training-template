package frc.DELib26.motors;

import com.ctre.phoenix6.swerve.utility.PhoenixPIDController;
import com.pathplanner.lib.config.PIDConstants;
import edu.wpi.first.math.controller.PIDController;

/**
 * Simple container for PID gains (kP, kI, kD).
 *
 * Stores numeric values as constants and provides factory
 * methods to create new WPILib {@link edu.wpi.first.math.controller.PIDController}
 * or CTRE {@link com.ctre.phoenix6.controls.PhoenixPIDController} instances
 * when needed. This avoids constructing controllers too early or reusing
 * the same stateful object across subsystems.
 */
public class PIDControllerContainer {
	public double kP = 0;
	public double kI = 0;
	public double kD = 0;

	public PIDControllerContainer withKP(double kP) {
		this.kP = kP;
		return this;
	}

	public PIDControllerContainer withKI(double kI) {
		this.kI = kI;
		return this;
	}

	public PIDControllerContainer withKD(double kD) {
		this.kD = kD;
		return this;
	}

	public PIDController toPIDController() {
		return new PIDController(this.kP, this.kI, this.kD);
	}

	public PhoenixPIDController toPhoenixPIDController() {
		return new PhoenixPIDController(this.kP, this.kI, this.kD);
	}

	public PIDConstants toPIDConstants() {
		return new PIDConstants(this.kP, this.kI, this.kD);
	}
}
