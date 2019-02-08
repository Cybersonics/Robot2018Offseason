package org.usfirst.frc.team103.subsystem;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import edu.wpi.first.wpilibj.command.Subsystem;
import org.usfirst.frc.team103.command.TeleopDrive;
import org.usfirst.frc.team103.robot.RobotMap;

import java.util.stream.DoubleStream;

public class Drive extends Subsystem {
	public static final double WHEEL_BASE_LENGTH = 20.875, WHEEL_BASE_WIDTH = 26.125;
	//XXX: FIX THIS BEFORE COMPETITION, WHEEL DIAMETER SHOULD BE 4" maybe
	public static final double WHEEL_DIAMETER = 3.95;
	public static final double MAX_SPEED = 36.0 * WHEEL_DIAMETER;
	public static final double STEER_DEGREES_PER_COUNT = 360.0 / 1024.0, DRIVE_INCHES_PER_COUNT = (WHEEL_DIAMETER * Math.PI) / (80.0 * 6.67);
	public static final double DEADZONE = 0.08;
	public static final double MAX_REVERSIBLE_SPEED_DIFFERENCE = 0.5 * MAX_SPEED;

	public double getAverageSpeed() {
		return DoubleStream.of(
				RobotMap.driveLeftFront.getSelectedSensorVelocity(0) * 10.0 * DRIVE_INCHES_PER_COUNT / MAX_SPEED,
				RobotMap.driveLeftRear.getSelectedSensorVelocity(0) * 10.0 * DRIVE_INCHES_PER_COUNT / MAX_SPEED,
				RobotMap.driveRightFront.getSelectedSensorVelocity(0) * 10.0 * DRIVE_INCHES_PER_COUNT / MAX_SPEED,
				RobotMap.driveRightRear.getSelectedSensorVelocity(0) * 10.0 * DRIVE_INCHES_PER_COUNT / MAX_SPEED
		).map(Math::abs).average().getAsDouble();
	}

	public void setNeutralMode(NeutralMode mode) {
		RobotMap.driveLeftFront.setNeutralMode(mode);
		RobotMap.driveLeftRear.setNeutralMode(mode);
		RobotMap.driveRightFront.setNeutralMode(mode);
		RobotMap.driveRightRear.setNeutralMode(mode);
	}

	public void stop(boolean usePID) {
		if (usePID) {
			RobotMap.driveLeftFront.set(ControlMode.Velocity, 0.0);
			RobotMap.driveLeftRear.set(ControlMode.Velocity, 0.0);
			RobotMap.driveRightFront.set(ControlMode.Velocity, 0.0);
			RobotMap.driveRightRear.set(ControlMode.Velocity, 0.0);
		} else {
			RobotMap.driveLeftFront.set(ControlMode.PercentOutput, 0.0);
			RobotMap.driveLeftRear.set(ControlMode.PercentOutput, 0.0);
			RobotMap.driveRightFront.set(ControlMode.PercentOutput, 0.0);
			RobotMap.driveRightRear.set(ControlMode.PercentOutput, 0.0);
		}
	}

	public void disable() {
		RobotMap.driveLeftFront.set(ControlMode.Disabled, 0.0);
		RobotMap.driveLeftRear.set(ControlMode.Disabled, 0.0);
		RobotMap.driveRightFront.set(ControlMode.Disabled, 0.0);
		RobotMap.driveRightRear.set(ControlMode.Disabled, 0.0);
	}

	public void swerveDrive(double strafe, double forward, double omega, boolean useDeadzone, boolean usePID) {
		if (useDeadzone) {
			strafe = (Math.abs(strafe) < DEADZONE ? 0.0 : strafe);
			forward = (Math.abs(forward) < DEADZONE ? 0.0 : forward);
			omega = (Math.abs(omega) < DEADZONE ? 0.0 : omega);
			
			if (strafe == 0.0 && forward == 0.0 && omega == 0.0) {
				if (usePID) {
					RobotMap.driveLeftFront.set(ControlMode.Velocity, 0.0);
					RobotMap.driveLeftRear.set(ControlMode.Velocity, 0.0);
					RobotMap.driveRightFront.set(ControlMode.Velocity, 0.0);
					RobotMap.driveRightRear.set(ControlMode.Velocity, 0.0);
				} else {
					RobotMap.driveLeftFront.set(ControlMode.PercentOutput, 0.0);
					RobotMap.driveLeftRear.set(ControlMode.PercentOutput, 0.0);
					RobotMap.driveRightFront.set(ControlMode.PercentOutput, 0.0);
					RobotMap.driveRightRear.set(ControlMode.PercentOutput, 0.0);
				}
				return;
			}
			
			if (usePID) {
				strafe = (strafe - Math.signum(strafe) * DEADZONE) / (1.0 - DEADZONE);
				forward = (forward - Math.signum(forward) * DEADZONE) / (1.0 - DEADZONE);
				omega = (omega - Math.signum(omega) * DEADZONE) / (1.0 - DEADZONE);
			}
		}
		
		strafe *= MAX_SPEED;   //Converting velocity to inches per second.
		forward *= MAX_SPEED;
		omega *= 2.0 * Math.PI;

		double A = strafe - omega * WHEEL_BASE_LENGTH / 2.0;
		double B = strafe + omega * WHEEL_BASE_LENGTH / 2.0;
		double C = forward - omega * WHEEL_BASE_WIDTH / 2.0;
		double D = forward + omega * WHEEL_BASE_WIDTH / 2.0;
		
		//Wheel 2
		double leftFrontSpeed = Math.hypot(B, D);
		double leftFrontAngle = Math.toDegrees(Math.atan2(B, D));
		
		//Wheel 1
		double rightFrontSpeed = Math.hypot(B, C);
		double rightFrontAngle = Math.toDegrees(Math.atan2(B, C));
		
		//Wheel 3
		double leftRearSpeed = Math.hypot(A, D);
		double leftRearAngle = Math.toDegrees(Math.atan2(A, D));
		
		//Wheel 4
		double rightRearSpeed = Math.hypot(A, C);
		double rightRearAngle = Math.toDegrees(Math.atan2(A, C));

		double maximumSpeed = DoubleStream.of(MAX_SPEED, leftFrontSpeed, rightFrontSpeed, leftRearSpeed, rightRearSpeed).max().getAsDouble() / MAX_SPEED;
		leftFrontSpeed /= maximumSpeed;
		rightFrontSpeed /= maximumSpeed;
		leftRearSpeed /= maximumSpeed;
		rightRearSpeed /= maximumSpeed;

		setSwerveModule(RobotMap.driveLeftFront, RobotMap.steerLeftFront, leftFrontAngle, leftFrontSpeed, usePID);
		setSwerveModule(RobotMap.driveRightFront, RobotMap.steerRightFront, rightFrontAngle, rightFrontSpeed, usePID);
		setSwerveModule(RobotMap.driveLeftRear, RobotMap.steerLeftRear, leftRearAngle, leftRearSpeed, usePID);
		setSwerveModule(RobotMap.driveRightRear, RobotMap.steerRightRear, rightRearAngle, rightRearSpeed, usePID);
	}

	private void setSwerveModule(TalonSRX drive, TalonSRX steer, double angle, double speed, boolean usePID) {
		// Get the current angle and speed for the module
		double currentAngle = steer.getSelectedSensorPosition(0) * STEER_DEGREES_PER_COUNT;
		double currentSpeed = drive.getSelectedSensorVelocity(0) * 10.0 * DRIVE_INCHES_PER_COUNT;

		// Calculate the number of degrees to turn assuming that speed is not reversed
		double angleDelta = Math.IEEEremainder(angle - currentAngle, 360.0);
		// Calculate the corresponding change in speed required
		double speedDifference = Math.abs(speed - currentSpeed);

		// Calculate the angle that requires the least amount of turning by allowing speed reversal
		double shortestAngleDelta = Math.IEEEremainder(angleDelta, 180.0);
		// If the previous calculation flipped the direction to turn, then the speed must be reversed
		double shortestSpeed = Math.signum(angleDelta) * Math.signum(shortestAngleDelta) * speed;
		// Calculate the change in speed when speed reversal is allowed
		double shortestSpeedDifference = Math.abs(shortestSpeed - currentSpeed);

		// If the change in speed required when using the angle that requires the least amount of turning is below the
		// speed reversal threshold, then always use that angle and corresponding speed
		// If the change in speed is above the reversal speed threshold, prefer the turning direction that results in
		// the least change in speed
		if (shortestSpeedDifference <= MAX_REVERSIBLE_SPEED_DIFFERENCE || shortestSpeedDifference <= speedDifference) {
			angleDelta = shortestAngleDelta;
			speed = shortestSpeed;
		}

		// Set the steering motor position and drive motor output accordingly
		steer.set(ControlMode.Position, (currentAngle + angleDelta) / STEER_DEGREES_PER_COUNT);
		if (usePID) {
			drive.set(ControlMode.Velocity, speed / DRIVE_INCHES_PER_COUNT / 10.0);
		} else {
			drive.set(ControlMode.PercentOutput, speed / MAX_SPEED);
		}
	}
	
	@Override
	protected void initDefaultCommand() {
		setDefaultCommand(new TeleopDrive());
	}
}

