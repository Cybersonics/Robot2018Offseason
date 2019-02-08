package org.usfirst.frc.team103.unused;

import org.usfirst.frc.team103.command.DriveTo;
import org.usfirst.frc.team103.robot.RobotMap;

import edu.wpi.first.wpilibj.command.CommandGroup;

public class RightSwitchRightScale extends CommandGroup {
	public RightSwitchRightScale() {
		requires(RobotMap.drive);
		addSequential(new DriveTo(-25.0, 145.0, 270.0, 0.8, 0.4));
		addSequential(new DriveTo(-25.0, 225.0, 215.0, 0.8, 0.8));
		//Vision Finding Cube
		addSequential(new DriveTo(-70.0, 195.0, 180.0, 0.5, 0.0));
		//Vision Getting Cube ^
		addSequential(new DriveTo(-25.0, 305.0, 270, 0.8, 0.0));
		
	}
}
