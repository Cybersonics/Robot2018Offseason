package org.usfirst.frc.team103.unused;

import edu.wpi.first.wpilibj.command.Command;

import static org.usfirst.frc.team103.robot.RobotMap.*;

import com.ctre.phoenix.motorcontrol.ControlMode;

public class ElevateTo extends Command {
	public static int ALLOWABLE_ERROR = 250, RAMP_ERROR = 500;
	
	private final int height;
	private final int button;
	
	public ElevateTo(int height, int button) {
		requires(elevator);
		this.height = height;
		this.button = button;
	}
	
	@Override
	protected void execute() {
		if (Math.abs(elevatorFront.getClosedLoopError(0)) < RAMP_ERROR) {
			elevatorFront.configPeakOutputForward(0.25, 0);
			elevatorFront.configPeakOutputReverse(-0.25, 0);
		} else {
			elevatorFront.configPeakOutputForward(1.0, 0);
			elevatorFront.configPeakOutputReverse(-1.0, 0);
		}
		elevatorFront.set(ControlMode.MotionMagic, height);
	}

	@Override
	protected boolean isFinished() {
		return !controller.getRawButton(button);
		//return !controller.getBButton();// && Math.abs(elevatorFront.getClosedLoopError(0)) < ALLOWABLE_ERROR;
	}

}
