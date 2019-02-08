package org.usfirst.frc.team103.command;

import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import org.usfirst.frc.team103.robot.Pathing;
import org.usfirst.frc.team103.robot.RobotMap;
import org.usfirst.frc.team103.robot.UltrasonicPositioning;
import org.usfirst.frc.team103.subsystem.Drive;

import java.awt.geom.Line2D;
import java.awt.geom.Point2D;
import java.util.List;

public class PathTo2 extends Command {
	public static final double POSITION_ACCELERATION = 0.8, HEADING_VELOCITY = 90.0, HEIGHT_VELOCITY = 8000.0;
	public static final double POSITION_CORRECTION_RAMP = 30.0, HEADING_CORRECTION_RAMP = 120.0;
	public static final double ALLOWABLE_POSITION_ERROR = 3.0, ALLOWABLE_HEADING_ERROR = 5.0;
	public static final int ALLOWABLE_HEIGHT_ERROR = 150;
	public static final double MIN_POSITION_SPEED = 0.1, MIN_POSITION_END_SPEED = 0.0;

	private final Point2D.Double targetPosition;
	private final double targetHeading;
	private final double cruiseSpeed, endSpeed;
	private final int cruiseHeight, endHeight;
	private final boolean useTurnBanking;

	private int segmentIndex, lastSegmentIndex;
	private PathSegment[] segments;
	private int heightTransitionIndex;
	private double heightTransitionOffset;
	private double currentSegmentDistance;
	
	public PathTo2(Point2D.Double targetPosition, double targetHeading, double cruiseSpeed, double endSpeed, int cruiseHeight, int endHeight) {
		this(targetPosition, targetHeading, cruiseHeight, endHeight, cruiseHeight, endHeight, true);
	}

	public PathTo2(Point2D.Double targetPosition, double targetHeading, double cruiseSpeed, double endSpeed, int cruiseHeight, int endHeight, boolean useTurnBanking) {
		this.targetPosition = targetPosition;
		this.targetHeading = targetHeading;
		this.cruiseSpeed = cruiseSpeed;
		this.endSpeed = endSpeed;
		this.cruiseHeight = cruiseHeight;
		this.endHeight = endHeight;
		this.useTurnBanking = useTurnBanking;

		requires(RobotMap.drive);
		requires(RobotMap.elevator);
	}

	@Override
	protected void initialize() {
		Point2D.Double startPosition = RobotMap.positioning.getPosition();
		if (startPosition.equals(targetPosition)) {
			startPosition = new Point2D.Double(startPosition.x + 0.1, startPosition.y);
		}
		double startSpeed = RobotMap.drive.getAverageSpeed();
		double startHeading = RobotMap.positioning.getHeading();
		double startHeight = RobotMap.elevator.getHeight();

		List<Point2D.Double> points = Pathing.shortestPath(startPosition, targetPosition);

		int pointCount = points.size();
		int lastPointIndex = pointCount - 1;

		double[] pointSpeeds = new double[pointCount];
		pointSpeeds[lastPointIndex] = endSpeed;

		for (int i = lastPointIndex - 1; i >= 0; i--) {
			Point2D.Double currentPoint = points.get(i);
			Point2D.Double nextPoint = points.get(i + 1);
			double distanceToNext = currentPoint.distance(nextPoint);
			double pointSpeed;
			if (i > 0) {
				Point2D.Double previousPoint = points.get(i - 1);
				double inAngle = Math.atan2(currentPoint.x - previousPoint.x, currentPoint.y - previousPoint.y);
				double outAngle = Math.atan2(nextPoint.x - currentPoint.x, nextPoint.y - currentPoint.y);
				double turnAngle = Math.IEEEremainder(outAngle - inAngle, 2.0 * Math.PI);
				pointSpeed = clamp(1.0 - Math.abs(turnAngle) / (0.6667 * Math.PI), MIN_POSITION_SPEED, cruiseSpeed);
			} else {
				pointSpeed = startSpeed;
			}
			pointSpeeds[i] = closestSpeedAllowingTransition(pointSpeed, pointSpeeds[i + 1], distanceToNext);
		}

		int segmentCount = pointCount - 1;
		segments = new PathSegment[segmentCount];
		segmentIndex = 0;
		lastSegmentIndex = segmentCount - 1;

		PathSegmentSpeed[] segmentSpeeds = new PathSegmentSpeed[segmentCount];

		for (int i = 0; i < segmentCount; i++) {
			Point2D.Double segmentStart = points.get(i);
			Point2D.Double segmentEnd = points.get(i + 1);
			Line2D.Double segmentLine = new Line2D.Double(segmentStart, segmentEnd);
			double segmentLength = segmentStart.distance(segmentEnd);

			double segmentStartSpeed = pointSpeeds[i];
			double segmentEndSpeed = pointSpeeds[i + 1];

			double startToEndTransitionLength = transitionLength(segmentStartSpeed, segmentEndSpeed);
			double startToCruiseTransitionLength = transitionLength(segmentStartSpeed, cruiseSpeed);
			double cruiseToEndTransitionLength = transitionLength(cruiseSpeed, segmentEndSpeed);

			double segmentCruiseSpeed;
			if (segmentLength <= startToEndTransitionLength) {
				segmentCruiseSpeed = segmentStartSpeed;
			} else if (segmentLength <= startToCruiseTransitionLength + cruiseToEndTransitionLength) {
				double triangleSpeed = startToCruiseTransitionLength < cruiseToEndTransitionLength ? segmentStartSpeed : segmentEndSpeed;
				segmentCruiseSpeed = closestSpeedAllowingTransition(cruiseSpeed, triangleSpeed, (segmentLength - startToEndTransitionLength) / 2.0);
			} else {
				segmentCruiseSpeed = cruiseSpeed;
			}

			segmentSpeeds[i] = new PathSegmentSpeed(segmentLine, segmentStartSpeed, segmentCruiseSpeed, segmentEndSpeed);
		}

		double[] pathDurations = new double[pointCount];
		pathDurations[0] = 0.0;
		for (int i = 1; i < pointCount; i++) {
			pathDurations[i] = pathDurations[i - 1] + segmentSpeeds[i - 1].duration;
		}
		double totalPathDuration = pathDurations[lastPointIndex];

		double startToEndHeightDuration = (double) Math.abs(endHeight - startHeight) / HEIGHT_VELOCITY;
		double startToCruiseHeightDuration = (double) Math.abs(cruiseHeight - startHeight) / HEIGHT_VELOCITY;
		double cruiseToEndHeightDuration = (double) Math.abs(endHeight - cruiseHeight) / HEIGHT_VELOCITY;

		double heightTransitionTime;
		if (startToEndHeightDuration >= totalPathDuration) {
			heightTransitionTime = 0.0;
		} else if (startToCruiseHeightDuration + cruiseToEndHeightDuration >= totalPathDuration) {
			heightTransitionTime = (startToCruiseHeightDuration + totalPathDuration - cruiseToEndHeightDuration) / 2.0;
		} else {
			heightTransitionTime = totalPathDuration - cruiseToEndHeightDuration;
		}

		for (int i = 0; i < segmentCount; i++) {
			double segmentEndTime = pathDurations[i + 1];
			if (segmentEndTime >= heightTransitionTime) {
				heightTransitionIndex = i;
				double segmentStartTime = pathDurations[i];
				double timeOffset = heightTransitionTime - segmentStartTime;
				PathSegmentSpeed segment = segmentSpeeds[i];
				if (timeOffset < segment.cruiseTransitionDuration) {
					heightTransitionOffset = 0.0;
				} else if (timeOffset < segment.duration - segment.endTransitionDuration) {
					heightTransitionOffset = segment.cruiseTransitionLength + (timeOffset - segment.cruiseTransitionDuration) * Drive.MAX_SPEED * segment.cruiseSpeed;
				} else {
					heightTransitionOffset = segment.length - segment.endTransitionLength;
				}
				break;
			}
		}

		double[] pointHeadings = new double[pointCount];
		pointHeadings[0] = startHeading;
		/*Line2D.Double firstSegmentLine = segmentSpeeds[0].line;
		double firstSegmentNormalAngle = (Math.toDegrees(Math.atan2(-(firstSegmentLine.y2 - firstSegmentLine.y1), firstSegmentLine.x2 - firstSegmentLine.x1)) + 360.0) % 360.0;
		double segmentNormalMultiplier = Math.abs(firstSegmentNormalAngle - startHeading) > 180.0 ? -1.0 : 1.0;*/

		for (int i = 0; i < segmentCount; i++) {
			PathSegmentSpeed segment = segmentSpeeds[i];
			double segmentStartHeading = pointHeadings[i];

			double segmentAngle = (Math.toDegrees(Math.atan2(segment.line.x2 - segment.line.x1, segment.line.y2 - segment.line.y1)) + 360.0) % 360.0;
			double segmentNormalAngle = (Math.toDegrees(Math.atan2(-(segment.line.y2 - segment.line.y1), segment.line.x2 - segment.line.x1)) + 360.0) % 360.0;
			if (Math.abs(Math.IEEEremainder(segmentNormalAngle - segmentStartHeading, 360.0)) > 90.0) {
				segmentNormalAngle = (segmentNormalAngle + 180.0) % 360.0;
			}
			double segmentCruiseHeading = segmentNormalAngle;
			if (Math.abs(Math.IEEEremainder(segmentNormalAngle, 90.0)) < UltrasonicPositioning.MAX_ANGLE_TO_WALL) {
				segmentCruiseHeading = (segmentCruiseHeading - Math.IEEEremainder(segmentNormalAngle, 90.0)) % 360.0;
			}

			double segmentEndHeading;
			if (i < lastSegmentIndex) {
				PathSegmentSpeed nextSegment = segmentSpeeds[i + 1];
				double nextSegmentAngle = (Math.toDegrees(Math.atan2(nextSegment.line.x2 - nextSegment.line.x1, nextSegment.line.y2 - nextSegment.line.y1)) + 360.0) % 360.0;
				double nextSegmentNormalAngle = (Math.toDegrees(Math.atan2(-(nextSegment.line.y2 - nextSegment.line.y1), nextSegment.line.x2 - nextSegment.line.x1)) + 360.0) % 360.0;
				if (Math.signum(Math.IEEEremainder(nextSegmentAngle - segmentAngle, 360.0)) != Math.signum(Math.IEEEremainder(nextSegmentNormalAngle - segmentNormalAngle, 360.0))) {
					nextSegmentNormalAngle = (nextSegmentNormalAngle + 180.0) % 360.0;
				}
				double nextSegmentCruiseHeading = nextSegmentNormalAngle;
				if (Math.abs(Math.IEEEremainder(nextSegmentNormalAngle, 90.0)) < UltrasonicPositioning.MAX_ANGLE_TO_WALL) {
					nextSegmentCruiseHeading = (nextSegmentCruiseHeading - Math.IEEEremainder(nextSegmentNormalAngle, 90.0)) % 360.0;
				}
				segmentEndHeading = (segmentCruiseHeading + Math.IEEEremainder(nextSegmentCruiseHeading - segmentCruiseHeading, 360.0) / 2.0 + 360.0) % 360.0;
			} else {
				segmentEndHeading = targetHeading;
				if (useTurnBanking && Math.abs(Math.IEEEremainder(segmentEndHeading - segmentCruiseHeading, 360.0)) > 90.0) {
					segmentEndHeading = (segmentEndHeading + 180.0) % 360.0;
				}
			}
			pointHeadings[i + 1] = segmentEndHeading;

			double startToEndHeadingDuration = Math.abs(Math.IEEEremainder(segmentEndHeading - segmentStartHeading, 360.0)) / HEADING_VELOCITY;
			double startToCruiseHeadingDuration = Math.abs(Math.IEEEremainder(segmentCruiseHeading - segmentStartHeading, 360.0)) / HEADING_VELOCITY;
			double cruiseToEndHeadingDuration = Math.abs(Math.IEEEremainder(segmentEndHeading - segmentCruiseHeading, 360.0)) / HEADING_VELOCITY;

			double segmentHeadingTransitionDistance;
			if (startToEndHeadingDuration > segment.duration || startToCruiseHeadingDuration + cruiseToEndHeadingDuration > segment.duration) {
				segmentHeadingTransitionDistance = 0.0;
			} else {
				if (cruiseToEndHeadingDuration < segment.endTransitionDuration) {
					segmentHeadingTransitionDistance = segment.length - segment.endTransitionLength;
				} else {
					segmentHeadingTransitionDistance = Math.max(segment.length - segment.endTransitionLength - (cruiseToEndHeadingDuration - segment.endTransitionDuration) * Drive.MAX_SPEED * segment.cruiseSpeed, 0.0);
				}
			}

			segments[i] = new PathSegment(segment.line, segment.startSpeed, segment.cruiseSpeed, segment.endSpeed, segmentStartHeading, segmentCruiseHeading, segmentEndHeading, segmentHeadingTransitionDistance);
		}
		
		/*for (PathSegmentSpeed s : segmentSpeeds) System.out.println(s);
		System.out.println();
		for (PathSegment s : segments) System.out.println(s);
		System.out.println();*/

		currentSegmentDistance = 0.0;
		
		RobotMap.elevator.setClimber(false);
	}

	@Override
	protected void execute() {
		Point2D.Double currentPosition = RobotMap.positioning.getPosition();
		double currentHeading = RobotMap.positioning.getHeading();
		PathSegment segment = segments[segmentIndex];
		boolean isLastSegment = segmentIndex == lastSegmentIndex;

		double distanceAlongSegment = Math.max(segment.distanceAlong(currentPosition), 0.0);
		if (!isLastSegment) {
			distanceAlongSegment = Math.max(distanceAlongSegment, currentSegmentDistance);
			currentSegmentDistance = distanceAlongSegment;
		}
		double distanceToSegment = segment.distanceTo(currentPosition);

		double travelSpeed = segment.calculateSpeed(currentPosition, isLastSegment);
		double correctionSpeed = clamp((isLastSegment ? cruiseSpeed : travelSpeed) * (distanceToSegment / POSITION_CORRECTION_RAMP), -1.0, 1.0);

		double travelHeading = useTurnBanking ? (distanceAlongSegment >= segment.headingTransitionDistance ? segment.endHeading : segment.cruiseHeading) : targetHeading;
		double errorHeading = Math.IEEEremainder(travelHeading - currentHeading, 360.0);
		double travelRotationAngle = Math.toRadians(currentHeading - segment.angle);

		double strafe = correctionSpeed * Math.cos(travelRotationAngle) - travelSpeed * Math.sin(travelRotationAngle);
		double forward = correctionSpeed * Math.sin(travelRotationAngle) + travelSpeed * Math.cos(travelRotationAngle);
		double omega = Math.min(Math.max(errorHeading / HEADING_CORRECTION_RAMP, -1.0), 1.0);
		int height = segmentIndex > heightTransitionIndex || (segmentIndex == heightTransitionIndex && distanceAlongSegment >= heightTransitionOffset) ? endHeight : cruiseHeight;

		SmartDashboard.putNumber("PathToStrafe", strafe);
		SmartDashboard.putNumber("PathToForward", forward);
		SmartDashboard.putNumber("PathToOmega", omega);
		SmartDashboard.putNumber("PathToHeight", height);

		RobotMap.drive.swerveDrive(strafe, forward, omega, false, true);
		RobotMap.elevator.setHeight(height);

		if (!isLastSegment && distanceAlongSegment >= segment.length) {
			System.out.println("Finished " + segments[segmentIndex]);
			segmentIndex++;
			currentSegmentDistance = 0.0;
		}
	}

	@Override
	protected boolean isFinished() {
		if (segmentIndex == lastSegmentIndex) {
			Point2D.Double currentPosition = RobotMap.positioning.getPosition();
			double currentHeading = RobotMap.positioning.getHeading();
			int currentHeight = RobotMap.elevator.getHeight();
			if (endSpeed == 0.0) {
				boolean positionOnTarget = currentPosition.distance(targetPosition) <= ALLOWABLE_POSITION_ERROR;
				boolean headingOnTarget = Math.abs(Math.IEEEremainder(targetHeading - currentHeading, 360.0)) <= ALLOWABLE_HEADING_ERROR;
				boolean elevatorOnTarget = Math.abs(endHeight - currentHeight) <= ALLOWABLE_HEIGHT_ERROR;
				return positionOnTarget && headingOnTarget && elevatorOnTarget;
			} else {
				PathSegment lastSegment = segments[segmentIndex];
				return Math.max(lastSegment.distanceAlong(currentPosition), 0.0) >= lastSegment.length;
			}
		} else {
			return false;
		}
	}

	@Override
	protected void end() {
		System.out.println("Finished last segment " + segments[lastSegmentIndex]);
		if (endSpeed == 0.0) {
			RobotMap.drive.stop(true);
		}
	}

	private static double clamp(double value, double min, double max) {
		return Math.min(Math.max(value, min), max);
	}

	private static double transitionLength(double fromSpeed, double toSpeed) {
		if (fromSpeed != toSpeed) {
			return Drive.MAX_SPEED * (toSpeed * toSpeed - fromSpeed * fromSpeed) / (2.0 * Math.signum(toSpeed - fromSpeed) * POSITION_ACCELERATION);
		} else {
			return 0.0;
		}
	}

	private static double minSpeedAllowingTransition(double toSpeed, double distance) {
		return Math.sqrt(Math.max(toSpeed * toSpeed - distance * (2.0 * POSITION_ACCELERATION) / Drive.MAX_SPEED, 0.0));
	}

	private static double maxSpeedAllowingTransition(double toSpeed, double distance) {
		return Math.sqrt(Math.min(toSpeed * toSpeed + distance * (2.0 * POSITION_ACCELERATION) / Drive.MAX_SPEED, 1.0));
	}

	private static double closestSpeedAllowingTransition(double speed, double toSpeed, double distance) {
		return Math.min(Math.max(speed, minSpeedAllowingTransition(toSpeed, distance)), maxSpeedAllowingTransition(toSpeed, distance));
	}

	private static class PathSegmentSpeed {
		private final Line2D.Double line;
		private final double startSpeed, cruiseSpeed, endSpeed;
		private final double cruiseAcceleration, endAcceleration;
		private final double cruiseTransitionLength, endTransitionLength;
		private final double cruiseTransitionDuration, endTransitionDuration;
		private final double length, duration;

		private PathSegmentSpeed(Line2D.Double line, double startSpeed, double cruiseSpeed, double endSpeed) {
			this.line = line;
			this.startSpeed = startSpeed;
			this.cruiseSpeed = cruiseSpeed;
			this.endSpeed = endSpeed;

			cruiseAcceleration = Math.signum(cruiseSpeed - startSpeed) * POSITION_ACCELERATION;
			endAcceleration = Math.signum(endSpeed - cruiseSpeed) * POSITION_ACCELERATION;
			cruiseTransitionLength = transitionLength(startSpeed, cruiseSpeed);
			endTransitionLength = transitionLength(cruiseSpeed, endSpeed);
			cruiseTransitionDuration = cruiseAcceleration == 0.0 ? 0.0 : (cruiseSpeed - startSpeed) / cruiseAcceleration;
			endTransitionDuration = endAcceleration == 0.0 ? 0.0 : (endSpeed - cruiseSpeed) / endAcceleration;
			length = line.getP1().distance(line.getP2());
			duration = cruiseTransitionDuration + Math.max((length - cruiseTransitionLength - endTransitionLength) / Drive.MAX_SPEED, 0.0) / cruiseSpeed + endTransitionDuration;
		}

		@Override
		public String toString() {
			return "PathSegmentSpeed [line=" + line.getP1() + "-" + line.getP2() + ", startSpeed=" + startSpeed + ", cruiseSpeed=" + cruiseSpeed
					+ ", endSpeed=" + endSpeed + ", cruiseAcceleration=" + cruiseAcceleration + ", endAcceleration="
					+ endAcceleration + ", cruiseTransitionLength=" + cruiseTransitionLength + ", endTransitionLength="
					+ endTransitionLength + ", cruiseTransitionDuration=" + cruiseTransitionDuration
					+ ", endTransitionDuration=" + endTransitionDuration + ", length=" + length + ", duration="
					+ duration + "]";
		}
		
		
	}

	private static class PathSegment {
		private final Line2D.Double line;
		private final double startSpeed, cruiseSpeed, endSpeed;
		private final double startHeading, cruiseHeading, endHeading;
		private final double headingTransitionDistance;
		private final double cruiseAcceleration, endAcceleration;
		private final double cruiseTransitionLength, endTransitionLength;
		private final double length, angle;

		private PathSegment(
				Line2D.Double line,
				double startSpeed, double cruiseSpeed, double endSpeed,
				double startHeading, double cruiseHeading, double endHeading, double headingTransitionDistance
		) {
			this.line = line;
			this.startSpeed = startSpeed;
			this.cruiseSpeed = cruiseSpeed;
			this.endSpeed = endSpeed;
			this.startHeading = startHeading;
			this.cruiseHeading = cruiseHeading;
			this.endHeading = endHeading;
			this.headingTransitionDistance = headingTransitionDistance;

			cruiseAcceleration = Math.signum(cruiseSpeed - startSpeed) * POSITION_ACCELERATION;
			endAcceleration = Math.signum(endSpeed - cruiseSpeed) * POSITION_ACCELERATION;
			cruiseTransitionLength = transitionLength(startSpeed, cruiseSpeed);
			endTransitionLength = transitionLength(cruiseSpeed, endSpeed);
			length = line.getP1().distance(line.getP2());
			angle = (Math.toDegrees(Math.atan2(line.x2 - line.x1, line.y2 - line.y1)) + 360.0) % 360.0;
		}

		private double distanceAlong(Point2D.Double position) {
			double segmentDot = (position.x - line.x1) * (line.x2 - line.x1) + (position.y - line.y1) * (line.y2 - line.y1);
			return segmentDot / length;
		}

		private double distanceTo(Point2D.Double position) {
			double segmentNormalDot = (position.x - line.x1) * -(line.y2 - line.y1) + (position.y - line.y1) * (line.x2 - line.x1);
			return segmentNormalDot / length;
		}

		private double calculateSpeed(Point2D.Double position, boolean isLastSegment) {
			double distanceAlongSegment = Math.max(distanceAlong(position), 0.0);
			double distancePastEndTransition = distanceAlongSegment - (length - endTransitionLength);
			if (distancePastEndTransition >= 0.0) {
				if (isLastSegment && endSpeed == 0.0) {
					double squaredSpeed = cruiseSpeed * cruiseSpeed + 2.0 * endAcceleration * distancePastEndTransition / Drive.MAX_SPEED;
					return Math.signum(squaredSpeed) * Math.sqrt(clamp(Math.abs(squaredSpeed), MIN_POSITION_END_SPEED, 1.0));
				} else {
					return Math.sqrt(clamp(cruiseSpeed * cruiseSpeed + 2.0 * endAcceleration * distancePastEndTransition / Drive.MAX_SPEED, MIN_POSITION_SPEED, 1.0));
				}
			} else if (distanceAlongSegment <= cruiseTransitionLength) {
				return Math.sqrt(clamp(startSpeed * startSpeed + 2.0 * cruiseAcceleration * distancePastEndTransition / Drive.MAX_SPEED, MIN_POSITION_SPEED, 1.0));
			} else {
				return cruiseSpeed;
			}
		}

		@Override
		public String toString() {
			return "PathSegment [line=" + line.getP1() + "-" + line.getP2() + ", startSpeed=" + startSpeed + ", cruiseSpeed=" + cruiseSpeed
					+ ", endSpeed=" + endSpeed + ", startHeading=" + startHeading + ", cruiseHeading=" + cruiseHeading
					+ ", endHeading=" + endHeading + ", headingTransitionDistance=" + headingTransitionDistance
					+ ", cruiseAcceleration=" + cruiseAcceleration + ", endAcceleration=" + endAcceleration
					+ ", cruiseTransitionLength=" + cruiseTransitionLength + ", endTransitionLength="
					+ endTransitionLength + ", length=" + length + ", angle=" + angle + "]";
		}
	}
}
