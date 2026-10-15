package frc.DELib26.util;

import java.util.ArrayList;
import java.util.List;

public abstract class VirtualSubsystem {
private static List<VirtualSubsystem> instances = new ArrayList<>();

public VirtualSubsystem() {
	instances.add(this);
}

/** This method is called periodically before the command scheduler. */
public abstract void periodic();

/** Run the periodic methods for all subsystems. */
public static void runAllPeriodic() {
	for (VirtualSubsystem instance : instances) {
	instance.periodic();
	}
}
}
