package org.usfirst.frc.team103.unused;

import org.usfirst.frc.team103.command.DriveTo;
import org.usfirst.frc.team103.robot.RobotMap;

import edu.wpi.first.wpilibj.command.CommandGroup;

public class Poke extends CommandGroup {
	public Poke() {
		requires(RobotMap.drive);

		addSequential(new DriveTo(0.0, -30.0, 0.0, 0.5, 0.5, 5.0, 10.0));
		addSequential(new DriveTo(10.0, -40.0, 0.0, 0.5, 0.5, 5.0, 10.0));
		for (int i = 0; i < 6; i++) {
			addSequential(new DriveTo(40.0 + 30.0 * (double) i, -40.0, 0.0, 0.5, 0.5, 5.0, 10.0));
			addSequential(new DriveTo(50.0 + 30.0 * (double) i, -50.0, 0.0, 0.5, 0.5, 5.0, 10.0));
			addSequential(new DriveTo(50.0 + 30.0 * (double) i, -60.0, 0.0, 0.5, 0.0, 5.0, 10.0));
			addSequential(new DriveTo(50.0 + 30.0 * (double) i, -50.0, 0.0, 0.5, 0.5, 5.0, 10.0));
			addSequential(new DriveTo(60.0 + 30.0 * (double) i, -40.0, 0.0, 0.5, 0.5, 5.0, 10.0));
		}
		addSequential(new DriveTo(230.0, -50.0, 0.0, 0.2, 0.0, 5.0, 10.0));
	}
}
