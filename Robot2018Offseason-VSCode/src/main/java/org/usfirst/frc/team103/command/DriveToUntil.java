package org.usfirst.frc.team103.command;

import java.util.function.BooleanSupplier;

public class DriveToUntil extends DriveTo {
	private final BooleanSupplier condition;
	
	public DriveToUntil(double targetX, double targetY, double targetHeading, double cruiseSpeed, BooleanSupplier condition) {
		this(targetX, targetY, targetHeading, cruiseSpeed, 0.0, ALLOWABLE_POSITION_ERROR, ALLOWABLE_HEADING_ERROR, condition);
	}
	
	public DriveToUntil(double targetX, double targetY, double targetHeading, double cruiseSpeed, double endSpeed, BooleanSupplier condition) {
		this(targetX, targetY, targetHeading, cruiseSpeed, endSpeed, ALLOWABLE_POSITION_ERROR, ALLOWABLE_HEADING_ERROR, condition);
	}

	public DriveToUntil(double targetX, double targetY, double targetHeading, double cruiseSpeed, double endSpeed,
			double allowablePositionError, double allowableHeadingError, BooleanSupplier condition) {
		super(targetX, targetY, targetHeading, cruiseSpeed, endSpeed, allowablePositionError, allowableHeadingError);
		this.condition = condition;
	}

	@Override
	protected boolean isFinished() {
		return super.isFinished() || condition.getAsBoolean();
	}
}
