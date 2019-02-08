package org.usfirst.frc.team103.command;

import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj.command.Command;

import static org.usfirst.frc.team103.robot.RobotMap.*;

import org.usfirst.frc.team103.subsystem.Elevator;

public class TeleopElevator extends Command {
	public static final double SPEED_DEADZONE = 0.05;
	
	private boolean holdSet;
	private int holdHeight;
	
	public TeleopElevator() {
		requires(elevator);
	}
	
	@Override
	protected void initialize() {
		holdSet = false;
		elevator.setClimber(false);
	}
	
	@Override
	protected void execute() {
		if (controller.getAButton()) {
			holdSet = false;
			elevator.setHeight(Elevator.CLEAR_GROUND_HEIGHT);
		} else if (controller.getBButton()) {
			holdSet = false;
			elevator.setHeight(25000);
		} else if (controller.getBumper(Hand.kLeft)) {
			holdSet = false;
			elevator.setSpeed(-0.2, false);
		} else {
			double speed = controller.getTriggerAxis(Hand.kRight) - controller.getTriggerAxis(Hand.kLeft);
			if (Math.abs(speed) > SPEED_DEADZONE) {
				holdSet = false;
				elevator.setSpeed(speed, true);
			} else {
				if (!holdSet) {
					holdHeight = elevator.getHeight();
					holdSet = true;
				}
				elevator.setHeight(holdHeight);
			}
		}
	}

	@Override
	protected boolean isFinished() {
		return false;
	}
}
