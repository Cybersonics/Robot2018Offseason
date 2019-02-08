package org.usfirst.frc.team103.util;

import java.lang.reflect.Field;
import java.util.Vector;

import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.command.CommandGroup;

public abstract class DynamicCommandGroup extends CommandGroup {
	protected abstract void dynamicInitialize();
	
	@Override
	protected final void initialize() {
		try {
			Field commandsField = CommandGroup.class.getDeclaredField("m_commands");
			commandsField.setAccessible(true);
			Vector<?> commands = ((Vector<?>) commandsField.get(this));
			commands.clear();
			Field locked = Command.class.getDeclaredField("m_locked");
			locked.setAccessible(true);
			locked.set(this, false);
			dynamicInitialize();
			locked.set(this, true);
		} catch (NoSuchFieldException | SecurityException | IllegalArgumentException | IllegalAccessException e) {
			e.printStackTrace();
		}
	}
}
