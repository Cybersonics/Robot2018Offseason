package org.usfirst.frc.team103.unused;

import org.usfirst.frc.team103.robot.RobotMap;

import com.ctre.phoenix.motorcontrol.ControlMode;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.command.WaitForChildren;

public class DeployArms extends Command {
	public static final double FREQUENCY = 3.0;
	
	private double endTime;
	
	@Override
	protected void initialize() {
		System.out.println("Deploying arms");
		endTime = Timer.getFPGATimestamp() + 2.0;
	}
	
	@Override
	protected void execute() {
		RobotMap.conveyor.set(ControlMode.PercentOutput, (int) (FREQUENCY * Timer.getFPGATimestamp()) % 2 == 0 ? -1.0 : 1.0);
	}

	@Override
	protected boolean isFinished() {
		return Timer.getFPGATimestamp() > endTime;
	}
	
	@Override
	protected void end() {
		RobotMap.conveyor.set(ControlMode.PercentOutput, 0.0);
		System.out.println("Arms hopefully deployed");
	}
}
