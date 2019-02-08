package org.usfirst.frc.team103.unused;

import org.usfirst.frc.team103.command.DriveTo;
import org.usfirst.frc.team103.robot.RobotMap;

import edu.wpi.first.wpilibj.command.CommandGroup;

public class MiddleCubes extends CommandGroup {
	private static final double SPEED = 0.5, EXCHANGE_X = -24.0;
	
	public MiddleCubes() {
		requires(RobotMap.drive);
		addSequential(new DriveTo(EXCHANGE_X, 0.0, 0.0, SPEED));
		//place cube
		addSequential(new DriveTo(0.0, 64.0, 0, SPEED));
		//pickup cube
		addSequential(new DriveTo(EXCHANGE_X, 0.0, 0.0, SPEED));
		//place cube
		addSequential(new DriveTo(0.0, 77.0, 0.0, SPEED));
		//pickup cube
		addSequential(new DriveTo(EXCHANGE_X, 0.0, 0.0, SPEED));
		//place cube
		addSequential(new DriveTo(24.0, 80.0, 315.0, SPEED));
		//pickup cube
		addSequential(new DriveTo(EXCHANGE_X, 0.0, 0.0, SPEED));
		//place cube
		addSequential(new DriveTo(-24.0, 80.0, 45.0, SPEED));
		//pickup cube
		addSequential(new DriveTo(EXCHANGE_X, 0.0, 0.0, SPEED));
		//place cube
		addSequential(new DriveTo(0.0, 88.0, 0.0, SPEED));
		//pickup cube
		addSequential(new DriveTo(EXCHANGE_X, 0.0, 0.0, SPEED));
		//place cube
	}
}
