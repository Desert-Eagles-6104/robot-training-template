package frc.DELib26.bindings;

import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.DELib26.controller.GenericController;

/**
 * Prints a line to the console whenever any controller input is pressed.
 * Neutral names are used so it works for Xbox and PlayStation.
 */
public final class ControllerDiagnosticsBindings extends Bindings {
	private final GenericController c;

	public ControllerDiagnosticsBindings(GenericController controller) {
		this.c = controller;
	}

	private static void on(Trigger t, String name) {
		t.onTrue(new PrintCommand("[ControllerTest] " + name));
	}

	@Override
	protected void binding() {
		// Face buttons (position-based)
		on(c.westButton(), "FACE LEFT (Xbox=X,  PS=Square)");
		on(c.eastButton(), "FACE RIGHT (Xbox=B,  PS=Circle)");
		on(c.northButton(), "FACE TOP (Xbox=Y,  PS=Triangle)");
		on(c.southButton(), "FACE BOTTOM (Xbox=A,  PS=Cross)");

		// D-Pad / POV (all 8 directions + center)
		on(c.povUp(), "DPAD UP (0)");
		on(c.povUpRight(), "DPAD UP-RIGHT (45)");
		on(c.povRight(), "DPAD RIGHT (90)");
		on(c.povDownRight(), "DPAD DOWN-RIGHT (135)");
		on(c.povDown(), "DPAD DOWN (180°)");
		on(c.povDownLeft(), "DPAD DOWN-LEFT (225)");
		on(c.povLeft(), "DPAD LEFT (270)");
		on(c.povUpLeft(), "DPAD UP-LEFT (315)");
		on(c.povCenter(), "DPAD CENTER (-1)");

		// Bumpers
		on(c.leftBumper(), "LEFT BUMPER  (LB / L1)");
		on(c.rightBumper(), "RIGHT BUMPER (RB / R1)");

		// Stick presses
		on(c.leftStick(), "LEFT STICK PRESS (L3)");
		on(c.rightStick(), "RIGHT STICK PRESS (R3)");

		// System / menu buttons
		on(c.view(), "VIEW (Xbox View / PS Create)");
		on(c.options(), "MENU (Xbox Menu / PS Options)");

		// Triggers as digital (axis-threshold helpers; default 0.5)
		on(c.leftTrigger(), "LEFT TRIGGER (>0.5) (LT / L2)");
		on(c.rightTrigger(), "RIGHT TRIGGER (>0.5) (RT / R2)");
	}
}
