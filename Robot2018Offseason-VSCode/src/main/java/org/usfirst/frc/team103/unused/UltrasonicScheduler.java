package org.usfirst.frc.team103.unused;

import static org.usfirst.frc.team103.robot.RobotMap.positioning;

import java.util.concurrent.ExecutorService;
import java.util.concurrent.Executors;
import java.util.concurrent.LinkedTransferQueue;
import java.util.concurrent.ThreadFactory;
import java.util.concurrent.TransferQueue;
import java.util.function.DoubleConsumer;

public class UltrasonicScheduler {
	public static final double ALLOWABLE_HEADING_OFFSET = 20.0;
	public static final long TIMEOUT = 70;
	
	public static enum UltrasonicDirection {
		NORTH(0.0), EAST(90.0), SOUTH(180.0), WEST(270.0);
		
		public final double angle;
		
		private UltrasonicDirection(double angle) {
			this.angle = angle;
		}
	}
	
	private static enum RobotSide {
		FRONT(0, 1), RIGHT(2, 3), REAR(4, 5), LEFT(6, 7);
		
		public final int sensorA, sensorB;
		
		private RobotSide(int sensorA, int sensorB) {
			this.sensorA = sensorA;
			this.sensorB = sensorB;
		}
		
		public static RobotSide nearest(double angle) {
			if (angle >= 315.0 || angle < 45.0) {
				return FRONT;
			} else if (angle >= 45.0 && angle < 135.0) {
				return RIGHT;
			} else if (angle >= 135.0 && angle < 225.0) {
				return REAR;
			} else {
				return LEFT;
			}
		}
	}
	
	private class UltrasonicRequest {
		public final int ultrasonicIndex;
		public final DoubleConsumer callback;
		
		public UltrasonicRequest(int ultrasonicIndex, DoubleConsumer callback) {
			this.ultrasonicIndex = ultrasonicIndex;
			this.callback = callback;
		}
	}
	
	private final UltrasonicImproved[] ultrasonics;
	private boolean[] ultrasonicQueued;
	private final TransferQueue<UltrasonicRequest> requestQueue;
	private final ExecutorService executor;
	
	public UltrasonicScheduler() {
		//ultrasonics = new UltrasonicImproved[] { ultrasonicFrontLeft, ultrasonicFrontRight, ultrasonicRightFront, ultrasonicRightRear, ultrasonicRearRight, ultrasonicRearLeft, ultrasonicLeftRear, ultrasonicLeftFront };
		ultrasonics = null;
		ultrasonicQueued = new boolean[ultrasonics.length];
		requestQueue = new LinkedTransferQueue<>();
		executor = Executors.newSingleThreadExecutor(new ThreadFactory() {
			@Override
			public Thread newThread(Runnable r) {
				Thread t = Executors.defaultThreadFactory().newThread(r);
				t.setDaemon(true);
				return t;
			}
		});
		executor.submit(() -> {
			while (true) {
				UltrasonicRequest request = requestQueue.take();
				UltrasonicImproved ultrasonic = ultrasonics[request.ultrasonicIndex];
				ultrasonic.ping();
				long pingTime = System.currentTimeMillis();
				long timeoutTime = pingTime + TIMEOUT;
				while (System.currentTimeMillis() < timeoutTime && !ultrasonic.isRangeValid()) {
					Thread.sleep(1);
				}
				if (ultrasonic.isRangeValid()) {
					double distance = ultrasonic.getRange();
					if (Double.isFinite(distance)) {
						request.callback.accept(distance);
					}
				}
				ultrasonicQueued[request.ultrasonicIndex] = false;
			}
		});
	}

	public void request(UltrasonicDirection direction, DoubleConsumer callback) {
		double heading = positioning.getHeading();
		if (heading % 90.0 < ALLOWABLE_HEADING_OFFSET || heading % 90.0 > 90.0 - ALLOWABLE_HEADING_OFFSET) {
			double relativeAngle = (360.0 + (direction.angle - heading)) % 360.0;
			RobotSide side = RobotSide.nearest(relativeAngle);
			if (!ultrasonicQueued[side.sensorA]) {
				requestQueue.add(new UltrasonicRequest(side.sensorA, callback));
				ultrasonicQueued[side.sensorA] = true;
			}
			if (!ultrasonicQueued[side.sensorB]) {
				requestQueue.add(new UltrasonicRequest(side.sensorB, callback));
				ultrasonicQueued[side.sensorB] = true;
			}
		}
	}
}
