package org.usfirst.frc.team103.command;

import org.usfirst.frc.team103.robot.RobotMap;

import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj.command.Command;

public class Climb extends Command {
	public Climb() {
		requires(RobotMap.elevator);
		setInterruptible(false);
	}
	
	@Override
	protected void initialize() {
		RobotMap.elevator.setClimber(true);
	}
	
	@Override
	protected void execute() {
		RobotMap.elevator.setSpeed(Math.min(-RobotMap.controller.getTriggerAxis(Hand.kLeft), 0.0), false);
	}

	@Override
	protected boolean isFinished() {
		return false;
	}
}
