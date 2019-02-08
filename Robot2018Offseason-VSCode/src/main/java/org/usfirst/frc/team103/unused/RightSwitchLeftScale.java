package org.usfirst.frc.team103.unused;

import org.usfirst.frc.team103.command.DriveTo;
import org.usfirst.frc.team103.robot.RobotMap;

import edu.wpi.first.wpilibj.command.CommandGroup;



public class RightSwitchLeftScale extends CommandGroup {

    public RightSwitchLeftScale() {
       requires(RobotMap.drive);
       
       addSequential(new DriveTo(-23.50, 145.00, 270.00, 0.8, 0.0));
       addSequential(new DriveTo(-35.00, 205.00, 247.00, 0.8, 0.3));
       //Below Command is Substitute for Vision Finding Cube
       addSequential(new DriveTo(-72.00, 192.00, 186.00, 0.3, 0.0));
       //Possibly Replace with Vision Finding Cube ^
       addSequential(new DriveTo(-250.00, 245.00, 0, 0.8, 0.5, 5.0, 5.0));
       addSequential(new DriveTo(-250.00, 290.00, 90.00, 0.3, 0.0, 3.0, 3.0));
    	
    }
}
