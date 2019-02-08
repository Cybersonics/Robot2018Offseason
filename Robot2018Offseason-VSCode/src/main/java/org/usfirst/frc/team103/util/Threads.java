package org.usfirst.frc.team103.util;

import java.util.concurrent.Executors;
import java.util.concurrent.ScheduledExecutorService;
import java.util.concurrent.ThreadFactory;
import java.util.concurrent.TimeUnit;

public class Threads {
	public static final int DAEMON_THREAD_COUNT = 4;
	
	private static final ScheduledExecutorService daemonExecutor = Executors.newScheduledThreadPool(DAEMON_THREAD_COUNT, new ThreadFactory() {
		@Override
		public Thread newThread(Runnable r) {
			Thread t = Executors.defaultThreadFactory().newThread(r);
			t.setDaemon(true);
			return t;
		}
	});
	
	public static void scheduleAtFixedRate(Runnable r, int periodMilliseconds) {
		daemonExecutor.scheduleAtFixedRate(r, 0, periodMilliseconds, TimeUnit.MILLISECONDS);
	}
}
