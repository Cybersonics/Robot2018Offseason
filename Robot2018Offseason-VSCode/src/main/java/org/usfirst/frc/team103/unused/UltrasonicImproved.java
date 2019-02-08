package org.usfirst.frc.team103.unused;

import edu.wpi.first.wpilibj.Counter;
import edu.wpi.first.wpilibj.DigitalOutput;
import edu.wpi.first.wpilibj.Timer;

public class UltrasonicImproved {
	private final DigitalOutput ping;
	private final Counter echo;
	private double lastPingTime;
	
	public UltrasonicImproved(int pingChannel, int echoChannel) {
		ping = new DigitalOutput(pingChannel);
		echo = new Counter(echoChannel);
		//echo.setMaxPeriod(0.005); // divided by 10 because of error in HAL_SetCounterMaxPeriod
		echo.setSemiPeriodMode(true);
		echo.setUpdateWhenEmpty(false);
		echo.setSamplesToAverage(1);
	}
	
	public void ping() {
		echo.reset();
		ping.pulse(0.000015);
		lastPingTime = Timer.getFPGATimestamp();
	}
	
	public boolean isReady() {
		//return echo.getStopped();
		return Timer.getFPGATimestamp() > lastPingTime + 0.050;
	}
	
	public boolean isRangeValid() {
		return echo.get() >= 2 && Double.isFinite(echo.getPeriod());
	}
	
	public int getCount() {
		return echo.get();
	}
	
	public double getRange() {
		return echo.getPeriod() * 1125.0 * 12.0 * 0.5;
	}
}
