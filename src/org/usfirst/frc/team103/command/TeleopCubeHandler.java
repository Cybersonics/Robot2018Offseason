package org.usfirst.frc.team103.command;

import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import static org.usfirst.frc.team103.robot.RobotMap.*;

public class TeleopCubeHandler extends Command {
	public static final double STICK_DEADZONE = 0.08;
	//XXX: Change to 1.0 for competition
	public static final double SPEED_MULTIPLIER = 1.0;
	
	public TeleopCubeHandler() {
		requires(cubeHandler);
	}
	
	@Override
	protected void execute() {
		int dpad = controller.getPOV();
		SmartDashboard.putNumber("DPad", dpad);
		if (dpad != -1) {
			cubeHandler.outtake(dpad, modulate(SPEED_MULTIPLIER));
		} else {
			double lx = controller.getX(Hand.kLeft);
			double ly = -controller.getY(Hand.kLeft);
			double rx = controller.getX(Hand.kRight);
			double ry = -controller.getY(Hand.kRight);
			double rightMagnitude = Math.hypot(rx, ry);
			double rightAngle = (Math.toDegrees(Math.atan2(rx, ry)) + 360.0) % 360.0;
			double leftMagnitude = Math.hypot(lx, ly);
			double leftAngle = (Math.toDegrees(Math.atan2(lx, ly)) + 360.0) % 360.0;
			if (rightMagnitude > STICK_DEADZONE) {
				// Intake direction is flipped so that a cube is taken in and moved in the direction the stick points
				cubeHandler.intake((rightAngle + 180.0) % 360.0, SPEED_MULTIPLIER * modulate(rightMagnitude));
			} else if (leftMagnitude > STICK_DEADZONE) {
				cubeHandler.outtake(leftAngle, SPEED_MULTIPLIER * modulate(leftMagnitude));
			} else {
				cubeHandler.hold();
			}
		}
	}
	
	private double modulate(double speed) {
		if (controller.getBumper(Hand.kRight)) {
			if (Math.random() > 0.9) {
				speed *= -1.0;
			} else {
				speed += 0.5 * speed * Math.sin(5.0 * 2.0 * Math.PI * Timer.getFPGATimestamp());
			}
		}
		return speed;
	}

	@Override
	protected boolean isFinished() {
		return false;
	}
}
