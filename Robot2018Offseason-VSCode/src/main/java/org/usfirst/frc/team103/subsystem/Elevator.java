package org.usfirst.frc.team103.subsystem;

import com.ctre.phoenix.motorcontrol.ControlMode;
import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import org.usfirst.frc.team103.command.TeleopElevator;
import org.usfirst.frc.team103.robot.RobotMap;

import static org.usfirst.frc.team103.robot.RobotMap.elevatorFront;

public class Elevator extends Subsystem {
	public static final int MIN_HEIGHT = 0, MAX_HEIGHT = 31000, MIN_ADJUSTMENT = 1000;
	public static final int RAMP_REGION = 5000, HOLD_REGION = 100;
	public static final double HOLD_PEAK_OUTPUT = 0.1667;

	public static final int CLEAR_GROUND_HEIGHT = 2000, CLEAR_OBSTACLE_HEIGHT = 9000;
	public static final double CLIMBER_ENGAGE = 0.3, CLIMBER_DISENGAGE = 0.55;

	public void disable() {
		elevatorFront.set(ControlMode.Disabled, 0.0);
	}

	public void setZeroHeight() {
		elevatorFront.setSelectedSensorPosition(0, 0, 0);
	}
	
	public void setClimber(boolean enabled) {
		RobotMap.climberClutch.set(enabled ? CLIMBER_ENGAGE : CLIMBER_DISENGAGE);
	}
	
	public void setSpeed(double speed, boolean useLimits) {
		if (useLimits) {
			int currentHeight = elevatorFront.getSelectedSensorPosition(0);
			int distanceToBottom = currentHeight - MIN_HEIGHT + MIN_ADJUSTMENT;
			int distanceToTop = MAX_HEIGHT - currentHeight;
			double maxDownSpeed = -1.0;
			double maxUpSpeed = 1.0;
			if (distanceToBottom < RAMP_REGION) {
				maxDownSpeed = Math.min((double) -distanceToBottom / (double) RAMP_REGION, 0.0);
			}
			if (distanceToTop < RAMP_REGION) {
				maxUpSpeed = Math.max((double) distanceToTop / (double) RAMP_REGION, 0.0);
			}
			speed = Math.min(Math.max(speed, maxDownSpeed), maxUpSpeed);
		}
		SmartDashboard.putNumber("ElevatorSpeed", speed);

		elevatorFront.configPeakOutputForward(1.0, 0);
		elevatorFront.configPeakOutputReverse(-1.0, 0);
		elevatorFront.configForwardSoftLimitEnable(useLimits, 0);
		elevatorFront.configReverseSoftLimitEnable(useLimits, 0);
		elevatorFront.set(ControlMode.PercentOutput, speed);
	}
	
	public void setHeight(int height) {
		int error = height - elevatorFront.getSelectedSensorPosition(0);
		SmartDashboard.putNumber("ElevatorError", error);
		if (Math.abs(error) < HOLD_REGION) {
			elevatorFront.configPeakOutputForward(HOLD_PEAK_OUTPUT, 0);
			elevatorFront.configPeakOutputReverse(-HOLD_PEAK_OUTPUT, 0);
		} else {
			elevatorFront.configPeakOutputForward(1.0, 0);
			elevatorFront.configPeakOutputReverse(-1.0, 0);
		}
		elevatorFront.configForwardSoftLimitEnable(true, 0);
		elevatorFront.configReverseSoftLimitEnable(true, 0);
		elevatorFront.set(ControlMode.MotionMagic, height);
	}
	
	public int getHeight() {
		return elevatorFront.getSelectedSensorPosition(0);
	}

	@Override
	protected void initDefaultCommand() {
		setDefaultCommand(new TeleopElevator());
	}
}
