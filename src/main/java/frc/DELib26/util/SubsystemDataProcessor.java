package frc.DELib26.util;

import edu.wpi.first.wpilibj.Notifier;

import java.util.List;
//og from 2910
public class SubsystemDataProcessor {
	public interface DataReaderAndLogger {
		void readAndLogDataFromIO();
	}

	public interface IODataRefresher {
		void refreshData();
	}

	public static final int LOOP_PERIOD_MS = 20;

	public static Notifier createAndStartSubsystemDataProcessor(
			DataReaderAndLogger dataReaderAndLogger,
			IODataRefresher ioRefresher,
			String name
		) {
		return createAndStartSubsystemDataProcessor(
				dataReaderAndLogger,
				List.of(ioRefresher),
				name
			);
	}

	public static Notifier createAndStartSubsystemDataProcessor(
			DataReaderAndLogger dataReaderAndLogger,
			List<IODataRefresher> ioRefreshers,
			String name
		) {
		Notifier notifier = new Notifier(() -> {
			for (IODataRefresher r : ioRefreshers) {
				r.refreshData();
			}
			dataReaderAndLogger.readAndLogDataFromIO();
		});
		notifier.setName("SubsystemDataProcessor-" + name);
		notifier.startPeriodic(LOOP_PERIOD_MS / 1000.0);
		return notifier;
	}

}