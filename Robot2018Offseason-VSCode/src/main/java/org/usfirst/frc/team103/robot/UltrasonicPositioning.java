package org.usfirst.frc.team103.robot;

import java.awt.Shape;
import java.awt.geom.Line2D;
import java.awt.geom.Point2D;
import java.awt.geom.Rectangle2D;

import org.usfirst.frc.team103.util.Threads;

import edu.wpi.first.wpilibj.Counter;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DigitalOutput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class UltrasonicPositioning {
	public static final int DIO_BASE = 0, ULTRASONIC_COUNT = 8;
	public static final int PING_PERIOD = 50;
	public static final double TRIGGER_DURATION = 0.000015;
	public static final double INCHES_PER_SECOND = 1125.0 * 12.0 * 0.5, MAX_RANGE = 120.0, MAX_PERIOD = MAX_RANGE / INCHES_PER_SECOND;
	public static final double MAX_ANGLE_TO_WALL = 20.0, MAX_RANGE_DEVIATION = 15.0, MAX_RANGE_DIFFERENCE_DEVIATION = 5.0;
	public static final double MERGE_WEIGHT = 0.2;
	public static final double SENSOR_FRONT_X = 12.875, SENSOR_FRONT_Y = 12.0, SENSOR_SIDE_X = 15.125, SENSOR_SIDE_Y = 9.75;
	
	public static final UltrasonicTarget[] TARGETS = {
		// left guard rail
		new UltrasonicTarget("LeftGuardRail", new Rectangle2D.Double(-162.0, 30.0, 90.0, 350.0), new Line2D.Double(-162.0, 380.0, -162.0, 30.0)),
		// right guard rail
		new UltrasonicTarget("RightGuardRail", new Rectangle2D.Double(72.0, 30.0, 90.0, 350.0), new Line2D.Double(162.0, 30.0, 162.0, 380.0)),
		// left driver station wall
		new UltrasonicTarget("LeftDriverStation", new Rectangle2D.Double(-132.0, 0.0, 70.0, 90.0), new Line2D.Double(-132.0, 0.0, -60.0, 0.0)),
		// right driver station wall
		new UltrasonicTarget("RightDriverStation", new Rectangle2D.Double(0.0, 0.0, 132.0, 90.0), new Line2D.Double(0.0, 0.0, 132.0, 0.0)),
		// left switch fence
		new UltrasonicTarget("LeftSwitchFence", new Rectangle2D.Double(-162.0, 150.0, 85.25, 36.0), new Line2D.Double(-76.75, 140.0, -76.75, 196.0)),
		// right switch fence
		new UltrasonicTarget("RightSwitchFence", new Rectangle2D.Double(76.75, 150.0, 85.25, 36.0), new Line2D.Double(76.75, 196.0, 76.75, 140.0)),
		// left scale wall
		new UltrasonicTarget("LeftScale", new Rectangle2D.Double(-51.75, 230.0, 31.75, 30.0), new Line2D.Double(-10.0, 315.0, -66.75, 315.0)),
		// right scale wall
		new UltrasonicTarget("RightScale", new Rectangle2D.Double(20.0, 230.0, 31.75, 30.0), new Line2D.Double(66.75, 315.0, 10.0, 315.0))
	};
	public static final UltrasonicTarget MIDDLE_SWITCH_FENCE = new UltrasonicTarget(
			"MiddleSwitchFence",
			new Rectangle2D.Double(-65.0, 211.0, 130.0, 49.0),
			new Line2D.Double(-76.75, 196.0, 76.75, 196.0)
	);

	private final DigitalOutput trigger;
	private final DigitalInput[] echoInputs = new DigitalInput[ULTRASONIC_COUNT];
	private final Counter[] echoes = new Counter[ULTRASONIC_COUNT];
	private final double[] ranges = new double[ULTRASONIC_COUNT];
	private final boolean[] rangesValid = new boolean[ULTRASONIC_COUNT];
	
	public UltrasonicPositioning() {
		trigger = new DigitalOutput(DIO_BASE + ULTRASONIC_COUNT);
		for (int i = 0; i < ULTRASONIC_COUNT; i++) {
			DigitalInput input = new DigitalInput(DIO_BASE + i);
			Counter counter = new Counter(input);
			counter.setSemiPeriodMode(true);
			counter.setUpdateWhenEmpty(false);
			counter.setSamplesToAverage(1);
			counter.reset();
			echoInputs[i] = input;
			echoes[i] = counter;
		}
		
		Threads.scheduleAtFixedRate(this::update, PING_PERIOD);
	}
	
	private void update() {
		for (int i = 0; i < ULTRASONIC_COUNT; i++) {
			Counter echo = echoes[i];
			double period = echo.getPeriod();
			if (echo.get() >= 2 && Double.isFinite(period) && period < MAX_PERIOD) {
				ranges[i] = period * INCHES_PER_SECOND;
				rangesValid[i] = true;
			} else {
				ranges[i] = MAX_RANGE;
				rangesValid[i] = false;
			}
			//SmartDashboard.putBoolean("Ultrasonic" + i + "Valid", rangesValid[i]);
			SmartDashboard.putNumber("Ultrasonic" + i, ranges[i]);
			echo.reset();
		}
		
		Point2D.Double position = RobotMap.positioning.getPosition();
		double heading = RobotMap.positioning.getHeading();
		StringBuilder correctionList = new StringBuilder("");
		String separator = "";
		for (UltrasonicTarget target : TARGETS) {
			if (target.availableArea.contains(position)) {
				double wallNormalAngle = Math.toDegrees(Math.atan2(target.wall.y2 - target.wall.y1, -(target.wall.x2 - target.wall.x1)));
				double robotToWallNormalAngle = ((wallNormalAngle - heading) % 360.0 + 360.0) % 360.0;
				if (Math.abs(Math.IEEEremainder(robotToWallNormalAngle, 90.0)) < MAX_ANGLE_TO_WALL) {
					RobotSide side = RobotSide.nearest(robotToWallNormalAngle);
					if (rangesValid[side.sensorA] && rangesValid[side.sensorB]) {
						double rangeA = ranges[side.sensorA];
						double rangeB = ranges[side.sensorB];
						double rangeDifference = rangeB - rangeA;
						double headingRadians = -Math.toRadians(heading);
						Point2D.Double rotatedSensorAPosition = new Point2D.Double(
							side.sensorAPosition.x * Math.cos(headingRadians) - side.sensorAPosition.y * Math.sin(headingRadians),
							side.sensorAPosition.x * Math.sin(headingRadians) + side.sensorAPosition.y * Math.cos(headingRadians)
						);
						Point2D.Double rotatedSensorBPosition = new Point2D.Double(
							side.sensorBPosition.x * Math.cos(headingRadians) - side.sensorBPosition.y * Math.sin(headingRadians),
							side.sensorBPosition.x * Math.sin(headingRadians) + side.sensorBPosition.y * Math.cos(headingRadians)
						);
						Point2D.Double sensorAPosition = new Point2D.Double(
							position.x + rotatedSensorAPosition.x,
							position.y + rotatedSensorAPosition.y
						);
						Point2D.Double sensorBPosition = new Point2D.Double(
							position.x + rotatedSensorBPosition.x,
							position.y + rotatedSensorBPosition.y
						);
						double expectedRangeA = target.wall.ptLineDist(sensorAPosition);
						double expectedRangeB = target.wall.ptLineDist(sensorBPosition);
						double expectedRangeDifference = expectedRangeB - expectedRangeA;
						double rangeDeviation = (Math.abs(rangeA - expectedRangeA) + Math.abs(rangeB - expectedRangeB)) / 2.0;
						double rangeDifferenceDeviation = Math.abs(rangeDifference - expectedRangeDifference);
						if (rangeDeviation < MAX_RANGE_DEVIATION && rangeDifferenceDeviation < MAX_RANGE_DIFFERENCE_DEVIATION) {
							double wallNormalX = -(target.wall.y2 - target.wall.y1);
							double wallNormalY = target.wall.x2 - target.wall.x1;
							double wallNormalLength = Math.hypot(wallNormalX, wallNormalY);
							Point2D.Double correctedAPosition = new Point2D.Double(
								sensorAPosition.x + wallNormalX / wallNormalLength * (rangeA - expectedRangeA),
								sensorAPosition.y + wallNormalY / wallNormalLength * (rangeA - expectedRangeA)
							);
							Point2D.Double correctedBPosition = new Point2D.Double(
								sensorBPosition.x + wallNormalX / wallNormalLength * (rangeB - expectedRangeB),
								sensorBPosition.y + wallNormalY / wallNormalLength * (rangeB - expectedRangeB)
							);
							double correctedARobotX = correctedAPosition.x - rotatedSensorAPosition.x;
							double correctedARobotY = correctedAPosition.y - rotatedSensorAPosition.y;
							double correctedBRobotX = correctedBPosition.x - rotatedSensorBPosition.x;
							double correctedBRobotY = correctedBPosition.y - rotatedSensorBPosition.y;
							double correctedRobotX = (correctedARobotX + correctedBRobotX) / 2.0;
							double correctedRobotY = (correctedARobotY + correctedBRobotY) / 2.0;
							RobotMap.positioning.mergePosition(correctedRobotX, correctedRobotY, MERGE_WEIGHT);

							correctionList.append(separator).append(target.label);
							separator = ", ";
						}
					}
				}
			}
		}

		UltrasonicTarget target = MIDDLE_SWITCH_FENCE;
		if (target.availableArea.contains(position)) {
			double wallNormalAngle = Math.toDegrees(Math.atan2(target.wall.y2 - target.wall.y1, -(target.wall.x2 - target.wall.x1)));
			double robotToWallNormalAngle = ((wallNormalAngle - heading) % 360.0 + 360.0) % 360.0;
			if (Math.abs(Math.IEEEremainder(robotToWallNormalAngle, 90.0)) < MAX_ANGLE_TO_WALL) {
				RobotSide side = RobotSide.nearest(robotToWallNormalAngle);
				if (rangesValid[side.sensorA] && rangesValid[side.sensorB]) {
					double headingRadians = -Math.toRadians(heading);
					Point2D.Double rotatedSensorAPosition = new Point2D.Double(
							side.sensorAPosition.x * Math.cos(headingRadians) - side.sensorAPosition.y * Math.sin(headingRadians),
							side.sensorAPosition.x * Math.sin(headingRadians) + side.sensorAPosition.y * Math.cos(headingRadians)
					);
					Point2D.Double rotatedSensorBPosition = new Point2D.Double(
							side.sensorBPosition.x * Math.cos(headingRadians) - side.sensorBPosition.y * Math.sin(headingRadians),
							side.sensorBPosition.x * Math.sin(headingRadians) + side.sensorBPosition.y * Math.cos(headingRadians)
					);
					Point2D.Double sensorAPosition = new Point2D.Double(
							position.x + rotatedSensorAPosition.x,
							position.y + rotatedSensorAPosition.y
					);
					Point2D.Double sensorBPosition = new Point2D.Double(
							position.x + rotatedSensorBPosition.x,
							position.y + rotatedSensorBPosition.y
					);
					double expectedRangeA = target.wall.ptLineDist(sensorAPosition);
					double expectedRangeB = target.wall.ptLineDist(sensorBPosition);
					double expectedRangeDifference = expectedRangeB - expectedRangeA;
					double rangeA = ranges[side.sensorA];
					double rangeB = ranges[side.sensorB];
					if (Math.abs(rangeA - expectedRangeA) > 7.5 && Math.abs(rangeA + 13.0 - expectedRangeA) < 7.5) {
						rangeA += 13.0;
					}
					if (Math.abs(rangeB - expectedRangeB) > 7.5 && Math.abs(rangeB + 13.0 - expectedRangeB) < 7.5) {
						rangeB += 13.0;
					}
					/*SmartDashboard.putNumber("RangeA", rangeA);
					SmartDashboard.putNumber("RangeB", rangeB);
					SmartDashboard.putNumber("ExpectedRangeA", expectedRangeA);
					SmartDashboard.putNumber("ExpectedRangeB", expectedRangeB);*/
					double rangeDifference = rangeB - rangeA;
					double rangeDeviation = (Math.abs(rangeA - expectedRangeA) + Math.abs(rangeB - expectedRangeB)) / 2.0;
					double rangeDifferenceDeviation = Math.abs(rangeDifference - expectedRangeDifference);
					if (rangeDeviation < MAX_RANGE_DEVIATION && rangeDifferenceDeviation < MAX_RANGE_DIFFERENCE_DEVIATION) {
						double wallNormalX = -(target.wall.y2 - target.wall.y1);
						double wallNormalY = target.wall.x2 - target.wall.x1;
						double wallNormalLength = Math.hypot(wallNormalX, wallNormalY);
						Point2D.Double correctedAPosition = new Point2D.Double(
								sensorAPosition.x + wallNormalX / wallNormalLength * (rangeA - expectedRangeA),
								sensorAPosition.y + wallNormalY / wallNormalLength * (rangeA - expectedRangeA)
						);
						Point2D.Double correctedBPosition = new Point2D.Double(
								sensorBPosition.x + wallNormalX / wallNormalLength * (rangeB - expectedRangeB),
								sensorBPosition.y + wallNormalY / wallNormalLength * (rangeB - expectedRangeB)
						);
						double correctedARobotX = correctedAPosition.x - rotatedSensorAPosition.x;
						double correctedARobotY = correctedAPosition.y - rotatedSensorAPosition.y;
						double correctedBRobotX = correctedBPosition.x - rotatedSensorBPosition.x;
						double correctedBRobotY = correctedBPosition.y - rotatedSensorBPosition.y;
						double correctedRobotX = (correctedARobotX + correctedBRobotX) / 2.0;
						double correctedRobotY = (correctedARobotY + correctedBRobotY) / 2.0;
						RobotMap.positioning.mergePosition(correctedRobotX, correctedRobotY, MERGE_WEIGHT);

						correctionList.append(separator).append(target.label);
					}
				}
			}
		}

		SmartDashboard.putString("UltrasonicCorrection", correctionList.toString());
		
		trigger.pulse(TRIGGER_DURATION);
	}
	
	private static class UltrasonicTarget {
		public final String label;
		public final Shape availableArea;
		// CCW orientation
		public final Line2D.Double wall;
		
		public UltrasonicTarget(String label, Shape availableArea, Line2D.Double targetWall) {
			this.label = label;
			this.availableArea = availableArea;
			this.wall = targetWall;
		}
	}
	
	private enum RobotSide {
		FRONT(5, 0, new Point2D.Double(-SENSOR_FRONT_X, SENSOR_FRONT_Y), new Point2D.Double(SENSOR_FRONT_X, SENSOR_FRONT_Y)),
		RIGHT(1, 2, new Point2D.Double(SENSOR_SIDE_X, SENSOR_SIDE_Y), new Point2D.Double(SENSOR_SIDE_X, -SENSOR_SIDE_Y)),
		REAR(3, 6, new Point2D.Double(SENSOR_FRONT_X, -SENSOR_FRONT_Y), new Point2D.Double(-SENSOR_FRONT_X, -SENSOR_FRONT_Y)),
		LEFT(7, 4, new Point2D.Double(-SENSOR_SIDE_X, -SENSOR_SIDE_Y), new Point2D.Double(-SENSOR_SIDE_X, SENSOR_SIDE_Y));
		
		public final int sensorA, sensorB;
		public final Point2D.Double sensorAPosition, sensorBPosition;
		
		RobotSide(int sensorA, int sensorB, Point2D.Double sensorAPosition, Point2D.Double sensorBPosition) {
			this.sensorA = sensorA;
			this.sensorB = sensorB;
			this.sensorAPosition = sensorAPosition;
			this.sensorBPosition = sensorBPosition;
		}
		
		public static RobotSide nearest(double angle) {
			if (angle >= 315.0 || angle < 45.0) {
				return FRONT;
			} else if (angle < 135.0) {
				return RIGHT;
			} else if (angle < 225.0) {
				return REAR;
			} else {
				return LEFT;
			}
		}
	}
}
