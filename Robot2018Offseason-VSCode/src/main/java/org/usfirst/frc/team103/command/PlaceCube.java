package org.usfirst.frc.team103.command;

import org.usfirst.frc.team103.robot.Autonomous;
import org.usfirst.frc.team103.robot.RobotMap;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.command.Command;

public class PlaceCube extends Command {
	public static final int ALLOWABLE_HEIGHT_ERROR = 500;
	public static final double PLACEMENT_TIMEOUT = 15.0, SENSED_PLACEMENT_TIMEOUT = 1.0;
	
	private final int height;
	private final double direction, speed;
	private final boolean placeOnLimit;
	
	private boolean isPlacing;
	private double placementEndTime;
	
	public PlaceCube(int height, double direction, double speed) {
		this(height, direction, speed, height >= Autonomous.SCALE_PLACE_HEIGHT);
	}
	
	public PlaceCube(int height, double direction, double speed, boolean placeOnLimit) {
		this.height = height;
		this.direction = direction;
		this.speed = speed;
		this.placeOnLimit = placeOnLimit;

		requires(RobotMap.elevator);
		requires(RobotMap.cubeHandler);
	}
	
	@Override
	protected void initialize() {
		isPlacing = false;
		RobotMap.elevator.setClimber(false);
		RobotMap.elevator.setHeight(height);
		RobotMap.cubeHandler.outtake(direction, 0.0);
	}
	
	@Override
	protected void execute() {
		boolean heightReached = Math.abs(RobotMap.elevator.getHeight() - height) < ALLOWABLE_HEIGHT_ERROR;
		if (placeOnLimit) {
			heightReached |= RobotMap.elevatorFront.getSensorCollection().isFwdLimitSwitchClosed();
		}
		if (!isPlacing && heightReached) {
			isPlacing = true;
			placementEndTime = Timer.getFPGATimestamp() + PLACEMENT_TIMEOUT;
		}
		if (isPlacing) {
			//boolean cubePresent = RobotMap.cubeHandler.outtake(direction, speed);
			RobotMap.cubeHandler.outtake(direction, speed);
			boolean cubePresent = RobotMap.cubeHandler.isAnyCubePresent();
			if (!cubePresent) {
				placementEndTime = Math.min(Timer.getFPGATimestamp() + SENSED_PLACEMENT_TIMEOUT, placementEndTime);
			}
		}
	}

	@Override
	protected boolean isFinished() {
		return isPlacing && Timer.getFPGATimestamp() > placementEndTime;
	}
	
	@Override
	protected void end() {
		RobotMap.cubeHandler.hold();
	}
}
