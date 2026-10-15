package frc.DELib26.util;

import java.util.Optional;

public class JavaUtils {
	public static boolean allPresent(Optional<?>... opts) {
		for (Optional<?> opt : opts) {
			if (opt.isEmpty()) {
				return false;
			}
		}
		return true;
	}

}
