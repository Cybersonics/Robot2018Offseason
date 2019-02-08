package org.usfirst.frc.team103.command;

import org.usfirst.frc.team103.robot.RobotMap;

import edu.wpi.first.wpilibj.command.Command;

public class TeleopDrive extends Command {
	public static boolean usePID = false;
	
	private double zeroHeading = 0.0;

	public TeleopDrive() {
		requires(RobotMap.drive);
	}

	@Override
	protected void initialize() {
		zeroHeading = 0.0;
	}

	@Override
	protected void execute() {
		/*double leftExponent = 2.0 - RobotMap.leftJoy.getRawAxis(2);
		double rightExponent = 2.0 - RobotMap.rightJoy.getRawAxis(2);
		
		double leftX = RobotMap.leftJoy.getX();
		double leftY = -RobotMap.leftJoy.getY();
		double rightX = RobotMap.rightJoy.getX();
		
		double strafe = Math.signum(leftX) * Math.pow(Math.abs(leftX), leftExponent);
		double forward = Math.signum(leftY) * Math.pow(Math.abs(leftY), leftExponent);
		double omega = Math.signum(rightX) * Math.pow(Math.abs(rightX), rightExponent);*/
		
		double strafe = RobotMap.leftJoy.getX();  
		double forward = -RobotMap.leftJoy.getY(); 
		double omega = RobotMap.rightJoy.getX();
		
		/*double frequency = 3.0;
		if (RobotMap.controller.getXButton()) {
			omega = Math.sin(Timer.getFPGATimestamp() * frequency * 2.0 * Math.PI);
		}
		if (RobotMap.controller.getYButton()) {
			forward = Math.sin(Timer.getFPGATimestamp() * frequency * 2.0 * Math.PI);
		}*/
		
		if (RobotMap.rightJoy.getTrigger()) {
			strafe *= 0.5;
			forward *= 0.5;
			omega *= 0.5;
		}
		
		if (RobotMap.leftJoy.getRawButton(7)) {
			zeroHeading = RobotMap.positioning.getHeading();
		}
		
		if (!RobotMap.leftJoy.getTrigger()) {
			double originCorrection = Math.toRadians(zeroHeading - RobotMap.positioning.getHeading());
    		double temp = forward * Math.cos(originCorrection) - strafe * Math.sin(originCorrection);
    		strafe = strafe * Math.cos(originCorrection) + forward * Math.sin(originCorrection);
    		forward = temp;
		}
		
		RobotMap.drive.swerveDrive(strafe, forward, omega, true, usePID);
	}

	@Override
	protected boolean isFinished() {
		return false;
	}
}
