package org.usfirst.frc.team103.unused;

import org.usfirst.frc.team103.command.DriveTo;

import edu.wpi.first.wpilibj.command.CommandGroup;

/**
 *
 */
public class DoubleRightSwitch extends CommandGroup {

    public DoubleRightSwitch() {
       addSequential(new DriveTo(-50.5, 143.0, 270.0, 0.7, 0.0, 0.3, 0.3));
       addSequential(new DriveTo(-50.5, 226.0, 270.0, 0.6, 0.3, 3.0, 3.0));
       addSequential(new DriveTo(-97.0, 188.0, 182.0, 0.4, 0.0, 6.0, 6.0));
       //^ Vision Finding Cube?
       addSequential(new DriveTo(-38.0, 186.0, 180.0, 0.7, 0.5, 3.0, 3.0));
       addSequential(new DriveTo(-50.0, 140.0, 270.0, 0.5, 0.0, 0.2, 0.2));
       //addSequential(new DriveTo(-))
      
    }
}
