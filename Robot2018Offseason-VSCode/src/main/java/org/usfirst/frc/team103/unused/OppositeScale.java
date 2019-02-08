package org.usfirst.frc.team103.unused;

import org.usfirst.frc.team103.command.DriveTo;
import org.usfirst.frc.team103.robot.RobotMap;

import edu.wpi.first.wpilibj.command.CommandGroup;

public class OppositeScale extends CommandGroup {
	
	private static final double SPEED_HIGH = 0.6, SPEED_LOW = 0.6, POSITION_ERROR = 5.0, ANGLE_ERROR = 10.0;
	
	public OppositeScale() {
		requires(RobotMap.drive);
		
		/*addSequential(new DriveTo(0.0, 30.0, 90.0, 0.5, 0.5, 5.0, 5.0));
		addSequential(new DriveTo(0.0, 60.0, 180.0, 0.5, 0.5, 5.0, 5.0));
		addSequential(new DriveTo(0.0, 90.0, 270.0, 0.5, 0.5, 5.0, 5.0));
		addSequential(new DriveTo(0.0, 120.0, 0.0, 0.5, 0.5, 5.0, 5.0));
		addSequential(new DriveTo(0.0, 150.0, 90.0, 0.5, 0.5, 5.0, 5.0));
		addSequential(new DriveTo(0.0, 180.0, 180.0, 0.5, 0.5, 5.0, 5.0));
		addSequential(new DriveTo(0.0, 210.0, 270.0, 0.5, 0.5, 5.0, 5.0));
		addSequential(new DriveTo(0.0, 240.0, 0.0, 0.5, 0.0));*/
		double dy = 20.0;
		addSequential(new DriveTo(0.0, 185.0 + dy, 0.0, SPEED_HIGH, SPEED_LOW, POSITION_ERROR, ANGLE_ERROR));
		addSequential(new DriveTo(-20.0, 205.0 + dy, 0.0, SPEED_LOW, SPEED_LOW, POSITION_ERROR, ANGLE_ERROR));
		addSequential(new DriveTo(-220.0, 205.0 + dy, 0.0, SPEED_HIGH, SPEED_LOW, POSITION_ERROR, ANGLE_ERROR));
		addSequential(new DriveTo(-240.0, 225.0 + dy, 0.0, SPEED_LOW, SPEED_LOW, POSITION_ERROR, ANGLE_ERROR));
		addSequential(new DriveTo(-240.0, 305.0, 90.0, SPEED_LOW, 0.0));
		addSequential(new DriveTo(-240.0, 225.0 + dy, 0.0, SPEED_LOW, SPEED_LOW, POSITION_ERROR, ANGLE_ERROR));
		addSequential(new DriveTo(-220.0, 205.0 + dy, 0.0, SPEED_LOW, SPEED_LOW, POSITION_ERROR, ANGLE_ERROR));
		addSequential(new DriveTo(-20.0, 205.0 + dy, 0.0, SPEED_HIGH, SPEED_LOW, POSITION_ERROR, ANGLE_ERROR));
		addSequential(new DriveTo(0.0, 185.0 + dy, 0.0, SPEED_LOW, SPEED_LOW, POSITION_ERROR, ANGLE_ERROR));
		addSequential(new DriveTo(0.0, 30.0, 0.0, SPEED_HIGH, 0.0));
	}

}
