package org.usfirst.frc.team103.command;

import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import static org.usfirst.frc.team103.robot.RobotMap.positioning;

import org.usfirst.frc.team103.robot.RobotMap;

public class DriveTo extends Command {
	
	public static final double ALLOWABLE_POSITION_ERROR = 3.0, ALLOWABLE_HEADING_ERROR = 5.0;
	public static final double MINIMUM_POSITION_SPEED = 0.05, MINIMUM_HEADING_SPEED = 0.05;
	public static final double POSITION_ERROR_BEFORE_RAMP_DOWN = 50.0, HEADING_ERROR_BEFORE_RAMP_DOWN = 90.0;
	
	private final double targetX, targetY;
	private final double targetHeading;
	private final double cruiseSpeed, endSpeed;
	private final double allowablePositionError, allowableHeadingError;
	
	private final double positionRampScale;
	
	public DriveTo(double targetX, double targetY, double targetHeading, double cruiseSpeed) {
		this(targetX, targetY, targetHeading, cruiseSpeed, 0.0, ALLOWABLE_POSITION_ERROR, ALLOWABLE_HEADING_ERROR);
	}
	
	public DriveTo(double targetX, double targetY, double targetHeading, double cruiseSpeed, double endSpeed) {
		this(targetX, targetY, targetHeading, cruiseSpeed, endSpeed, ALLOWABLE_POSITION_ERROR, ALLOWABLE_HEADING_ERROR);
	}
	
	public DriveTo(
			double targetX, double targetY, double targetHeading,
			double cruiseSpeed, double endSpeed,
			double allowablePositionError, double allowableHeadingError
	) {
		this.targetX = targetX;
		this.targetY = targetY;
		this.targetHeading = targetHeading;
		this.cruiseSpeed = cruiseSpeed;
		this.endSpeed = endSpeed;
		this.allowablePositionError = allowablePositionError;
		this.allowableHeadingError = allowableHeadingError;
		
		this.positionRampScale = (cruiseSpeed - endSpeed) / Math.pow(POSITION_ERROR_BEFORE_RAMP_DOWN, 2.0);
		
		requires(RobotMap.drive);
	}
	
	@Override
	protected void initialize() {
		System.out.println("Starting DriveTo(" + targetX + ", " + targetY + ")");
	}
	
	@Override
	protected void execute() {
		double currentX = positioning.getX();
		double currentY = positioning.getY();
		double currentHeading = positioning.getHeading();
		
		double errorX = targetX - currentX;
		double errorY = targetY - currentY;
		double errorHeading = targetHeading - currentHeading;
		if (Math.abs(errorHeading) > 180.0) {
			errorHeading -= 360.0 * Math.signum(errorHeading);
		}
		double distanceToTarget = Math.hypot(errorX, errorY);

		/*double speed = (errorPosition > POSITION_ERROR_BEFORE_RAMP_DOWN ? cruiseSpeed : 
				endSpeed + errorPosition * (cruiseSpeed - endSpeed) / POSITION_ERROR_BEFORE_RAMP_DOWN);*/
		
		//(0, endSpeed) to (POSITION_ERROR_BEFORE_RAMP_DOWN, maxSpeed), quadratic interpolation
		double speed = (distanceToTarget > POSITION_ERROR_BEFORE_RAMP_DOWN ? cruiseSpeed :
			cruiseSpeed - Math.pow(POSITION_ERROR_BEFORE_RAMP_DOWN - distanceToTarget, 2.0) * positionRampScale);
		double speedX = speed * errorX / distanceToTarget;
		double speedY = speed * errorY / distanceToTarget;
		/*SmartDashboard.putNumber("Speed", speed);
		SmartDashboard.putNumber("SpeedX", speedX);
		SmartDashboard.putNumber("SpeedY", speedY);*/
		double omega = Math.min(Math.max(errorHeading / HEADING_ERROR_BEFORE_RAMP_DOWN, -1.0), 1.0);
		
		double fieldHeadingCorrection = -Math.toRadians(currentHeading);
		double forward = speedY * Math.cos(fieldHeadingCorrection) - speedX * Math.sin(fieldHeadingCorrection);
		double strafe = speedX * Math.cos(fieldHeadingCorrection) + speedY * Math.sin(fieldHeadingCorrection);

		if (Math.abs(strafe) < MINIMUM_POSITION_SPEED) {
			strafe = Math.signum(strafe) * MINIMUM_POSITION_SPEED;
		}
		if (Math.abs(forward) < MINIMUM_POSITION_SPEED) {
			forward = Math.signum(forward) * MINIMUM_POSITION_SPEED;
		}
		//double minOmega = (Math.hypot(strafe, forward) < OMEGA_MINIMUM_CUTOFF ? MINIMUM_HEADING_SPEED : 0.0);
		//if (Math.abs(omega) < MINIMUM_HEADING_SPEED) omega = Math.signum(omega) * minOmega;

		SmartDashboard.putNumber("DriveToStrafe", strafe);
		SmartDashboard.putNumber("DriveToForward", forward);
		SmartDashboard.putNumber("DriveToOmega", omega);
		
		RobotMap.drive.swerveDrive(strafe, forward, omega, false, true);
	}

	@Override
	protected boolean isFinished() {
		double positionError = Math.hypot(targetX - positioning.getX(), targetY - positioning.getY());
		double headingError = targetHeading - positioning.getHeading();
		if (Math.abs(headingError) > 180.0) {
			headingError -= 360.0 * Math.signum(headingError);
		}
		return positionError < allowablePositionError && headingError < allowableHeadingError;
	}

	@Override
	protected void end() {
		System.out.println("Ending DriveTo(" + targetX + ", " + targetY + ")");
		if (endSpeed == 0.0) {
			RobotMap.drive.swerveDrive(0.0, 0.0, 0.0, false, true);
		}
	}
}
