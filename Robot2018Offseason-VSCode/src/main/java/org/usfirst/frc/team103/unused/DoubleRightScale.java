package org.usfirst.frc.team103.unused;

import org.usfirst.frc.team103.command.DriveTo;
import org.usfirst.frc.team103.robot.RobotMap;

import edu.wpi.first.wpilibj.command.CommandGroup;

public class DoubleRightScale extends CommandGroup {
	public DoubleRightScale() {
		requires(RobotMap.drive);
		
		addSequential(new DriveTo(-5.0, 305.0, 270.0, 0.8, 0.0));
		addSequential(new DriveTo(-25.0, 225.0, 215.0, 0.5, 0.1));
		//Vision Finding Cube
		addSequential(new DriveTo(-75.0, 193.0, 180.0, 0.5, 0.0));
		//Vision Getting Cube ^
		//addSequential(new DriveTo(-40.0, 225.0, 225.0, 0.6, 0.8));
		addSequential(new DriveTo(-5.0, 305.0, 270.0, 0.8, 0.0));
		
		
	}
	
	
	

}
