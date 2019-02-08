package org.usfirst.frc.team103.command;

import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import org.usfirst.frc.team103.robot.RobotMap;
import org.usfirst.frc.team103.subsystem.Drive;

import java.awt.geom.Line2D;
import java.awt.geom.Point2D;

public class DriveTo2 extends Command {
	public static final double ACCELERATION = 1.0;
	public static final double POSITION_CORRECTION_RAMP = 30.0, HEADING_CORRECTION_RAMP = 90.0;
	public static final double ALLOWABLE_POSITION_ERROR = 3.0, ALLOWABLE_HEADING_ERROR = 5.0;

	private final Point2D.Double targetPosition;
	private final double targetHeading;
	private final double cruiseSpeed, endSpeed;
	private final double endAcceleration;
	private final double transitionLength;

	private Line2D.Double path;
	private double pathAngle, pathLength;

	public DriveTo2(Point2D.Double targetPosition, double targetHeading, double cruiseSpeed, double endSpeed) {
		this.targetPosition = targetPosition;
		this.targetHeading = targetHeading;
		this.cruiseSpeed = cruiseSpeed;
		this.endSpeed = endSpeed;

		endAcceleration = Math.signum(endSpeed - cruiseSpeed) * ACCELERATION;
		transitionLength = Drive.MAX_SPEED * (endSpeed * endSpeed - cruiseSpeed * cruiseSpeed) / (2.0 * endAcceleration);

		requires(RobotMap.drive);
	}

	@Override
	protected void initialize() {
		Point2D.Double currentPosition = RobotMap.positioning.getPosition();
		if (currentPosition.equals(targetPosition)) {
			currentPosition = new Point2D.Double(currentPosition.x + 0.1, currentPosition.y);
		}
		path = new Line2D.Double(currentPosition, targetPosition);
		pathAngle = Math.toDegrees(Math.atan2(path.x2 - path.x1, path.y2 - path.y1));
		pathLength = currentPosition.distance(targetPosition);
	}

	@Override
	protected void execute() {
		Point2D.Double currentPosition = RobotMap.positioning.getPosition();
		double currentHeading = RobotMap.positioning.getHeading();

		double pathDot = (currentPosition.x - path.x1) * (path.x2 - path.x1) + (currentPosition.y - path.y1) * (path.y2 - path.y1);
		double pathNormalDot = (currentPosition.x - path.x1) * -(path.y2 - path.y1) + (currentPosition.y - path.y1) * (path.x2 - path.x1);
		double distanceAlongPath = pathDot / pathLength;
		double distancePastTransition = distanceAlongPath - (pathLength - transitionLength);
		double distanceToPath = pathNormalDot / pathLength;

		double travelSpeed;
		if (distancePastTransition >= 0.0) {
			if (endSpeed == 0.0) {
				double squaredSpeed = cruiseSpeed * cruiseSpeed + 2.0 * endAcceleration * distancePastTransition / Drive.MAX_SPEED;
				travelSpeed = Math.signum(squaredSpeed) * Math.sqrt(Math.min(Math.abs(squaredSpeed), 1.0));
			} else {
				travelSpeed = Math.sqrt(Math.min(Math.max(cruiseSpeed * cruiseSpeed + 2.0 * endAcceleration * distancePastTransition / Drive.MAX_SPEED, 0.0), 1.0));
			}
		} else {
			travelSpeed = cruiseSpeed;
		}
		double correctionSpeed = Math.min(Math.max(cruiseSpeed * (distanceToPath / POSITION_CORRECTION_RAMP), -1.0), 1.0);

		double errorHeading = Math.IEEEremainder(targetHeading - currentHeading, 360.0);
		double travelRotationAngle = Math.toRadians(currentHeading - pathAngle);

		double strafe = correctionSpeed * Math.cos(travelRotationAngle) - travelSpeed * Math.sin(travelRotationAngle);
		double forward = correctionSpeed * Math.sin(travelRotationAngle) + travelSpeed * Math.cos(travelRotationAngle);
		double omega = Math.min(Math.max(errorHeading / HEADING_CORRECTION_RAMP, -1.0), 1.0);

		SmartDashboard.putNumber("DriveToStrafe", strafe);
		SmartDashboard.putNumber("DriveToForward", forward);
		SmartDashboard.putNumber("DriveToOmega", omega);

		RobotMap.drive.swerveDrive(strafe, forward, omega, false, true);
	}

	@Override
	protected boolean isFinished() {
		Point2D.Double currentPosition = RobotMap.positioning.getPosition();
		double currentHeading = RobotMap.positioning.getHeading();
		if (endSpeed == 0.0) {
			boolean positionOnTarget = currentPosition.distance(targetPosition) <= ALLOWABLE_POSITION_ERROR;
			boolean headingOnTarget = Math.abs(Math.IEEEremainder(targetHeading - currentHeading, 360.0)) <= ALLOWABLE_HEADING_ERROR;
			return positionOnTarget && headingOnTarget;
		} else {
			double pathDot = (currentPosition.x - path.x1) * (path.x2 - path.x1) + (currentPosition.y - path.y1) * (path.y2 - path.y1);
			double distanceAlongPath = pathDot / pathLength;
			return distanceAlongPath >= pathLength;
		}
	}

	@Override
	protected void end() {
		if (endSpeed == 0.0) {
			RobotMap.drive.stop(true);
		}
	}
}
