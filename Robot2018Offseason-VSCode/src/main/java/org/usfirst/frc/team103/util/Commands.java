package org.usfirst.frc.team103.util;

import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.command.CommandGroup;
import edu.wpi.first.wpilibj.command.InstantCommand;

public class Commands {
	public static Command instant(Runnable initialize) {
		return instant(initialize, false);
	}

	public static Command instant(Runnable initialize, boolean runWhenDisabled) {
		return new InstantCommand() {
			{
				setRunWhenDisabled(runWhenDisabled);
			}
			@Override
			protected void initialize() {
				initialize.run();
			}
		};
	}
	
	public static CommandGroup sequential(Command... commands) {
		return new CommandGroup() {
			{
				for (Command command : commands) {
					addSequential(command);
				}
			}
		};
	}

	public static CommandGroup parallel(Command... commands) {
		return new CommandGroup() {
			{
				for (Command command : commands) {
					addParallel(command);
				}
			}
		};
	}
}
