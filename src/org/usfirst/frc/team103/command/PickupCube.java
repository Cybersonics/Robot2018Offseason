package org.usfirst.frc.team103.command;

import java.awt.geom.Point2D;

import org.usfirst.frc.team103.robot.RobotMap;

import edu.wpi.first.wpilibj.command.CommandGroup;

public class PickupCube extends CommandGroup {
	public static final double APPROACH_SPEED = 0.3, PICKUP_SPEED = 0.15;
	
	private final Point2D.Double cubePosition;
	private final boolean cubeForward;
	private final double robotHeading;
	
	public PickupCube(Point2D.Double cubePosition, boolean cubeForward, double robotHeading) {
		this.cubePosition = cubePosition;
		this.cubeForward = cubeForward;
		this.robotHeading = robotHeading;
		
		double multiplier = cubeForward ? 1.0 : -1.0;
		//double heading = cubeForward ? 0.0 : 180.0;
		double heading = robotHeading;
			
		addSequential(new DriveTo2(new Point2D.Double(cubePosition.x - multiplier * 10.0, cubePosition.y - multiplier * 26.0), heading + 15.0, APPROACH_SPEED, PICKUP_SPEED));
		addSequential(new DriveTo2(new Point2D.Double(cubePosition.x, cubePosition.y - multiplier * 21.0), heading + 10.0, PICKUP_SPEED, PICKUP_SPEED));
		addSequential(new DriveTo2(new Point2D.Double(cubePosition.x + multiplier * 10.0, cubePosition.y - multiplier * 21.0), heading - 15.0, PICKUP_SPEED, PICKUP_SPEED));
		addSequential(new DriveTo2(new Point2D.Double(cubePosition.x, cubePosition.y - multiplier * 31.0), heading, APPROACH_SPEED, 0.0));
		
		requires(RobotMap.elevator);
		requires(RobotMap.cubeHandler);
	}
	
	@Override
	protected void initialize() {
		RobotMap.elevator.setHeight(0);
		RobotMap.cubeHandler.intake(cubeForward ? 0.0 : 180.0, 0.5);
		System.out.println("Picking up cube from " + cubePosition + ", cubeForward=" + cubeForward + ", useRobotFront=" + robotHeading);
	}
}
