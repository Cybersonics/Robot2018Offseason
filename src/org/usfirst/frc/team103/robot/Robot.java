package org.usfirst.frc.team103.robot;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.buttons.JoystickButton;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.command.Scheduler;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import org.usfirst.frc.team103.command.Climb;
import org.usfirst.frc.team103.command.PathTo2;
import org.usfirst.frc.team103.command.PickupCube;
import org.usfirst.frc.team103.command.PlaceCube;
import org.usfirst.frc.team103.command.TeleopDrive;
import org.usfirst.frc.team103.util.Commands;

import java.awt.geom.Point2D;

public class Robot extends TimedRobot {
	private Command autonomousCommand;

	@Override
	public void robotInit() {
		RobotMap.init();

        /*new JoystickButton(RobotMap.leftJoy, 3).whenPressed(Commands.instant(() -> {
			RobotMap.positioning.setPosition(-60.0, 235.0);
        }, true));*/
        /*new JoystickButton(RobotMap.leftJoy, 6).whenPressed(Commands.instant(() -> {
			RobotMap.positioning.setPosition(103.0, 15.0);
        }, true));*/
        new JoystickButton(RobotMap.leftJoy, 7).whenPressed(Commands.instant(() -> {
        	System.out.println("Field-centric reset button pressed!");
			RobotMap.positioning.setFieldZeroHeading();
        }, true));
        
        //new JoystickButton(RobotMap.controller, 8).whenPressed(new Climb());
        SmartDashboard.putNumber("PathTestX", SmartDashboard.getNumber("PathTestX", -103.0));
        SmartDashboard.putNumber("PathTestY", SmartDashboard.getNumber("PathTestY", 220.0));
        SmartDashboard.putNumber("PathTestHeading", SmartDashboard.getNumber("PathTestHeading", 0.0));
        SmartDashboard.putNumber("PathTestCruiseSpeed", SmartDashboard.getNumber("PathTestCruiseSpeed", 0.5));
        SmartDashboard.putNumber("PathTestEndSpeed", SmartDashboard.getNumber("PathTestEndSpeed", 0.0));
        SmartDashboard.putNumber("PathTestCruiseHeight", SmartDashboard.getNumber("PathTestCruiseHeight", 9000));
        SmartDashboard.putNumber("PathTestEndHeight", SmartDashboard.getNumber("PathTestEndHeight", 9000));
        SmartDashboard.putBoolean("PathTestUseBanking", SmartDashboard.getBoolean("PathTestUseBanking", true));
        SmartDashboard.putNumber("CubePickupX", SmartDashboard.getNumber("CubePickupX", 0.0));
        SmartDashboard.putNumber("CubePickupY", SmartDashboard.getNumber("CubePickupY", 80.0));
        SmartDashboard.putBoolean("CubePickupCubeForward", SmartDashboard.getBoolean("CubePickupCubeForward", true));
        SmartDashboard.putNumber("CubePickupRobotHeading", SmartDashboard.getNumber("CubePickupRobotHeading", 0.0));
        
        //new JoystickButton(RobotMap.leftJoy, 8).whenPressed(Commands.instant(() -> new PlaceCube(RobotMap.elevator.getHeight(), 0.0, 1.0).start()));
        /*new JoystickButton(RobotMap.leftJoy, 8).whenPressed(new TeleopDrive());
        new JoystickButton(RobotMap.leftJoy, 9).whenPressed(Commands.instant(() -> {
        	new PathTo2(
        			new Point2D.Double(SmartDashboard.getNumber("PathTestX", -103.0), SmartDashboard.getNumber("PathTestY", 220.0)),
        			SmartDashboard.getNumber("PathTestHeading", 0.0),
        			SmartDashboard.getNumber("PathTestCruiseSpeed", 0.5), SmartDashboard.getNumber("PathTestEndSpeed", 0.0),
        			(int) SmartDashboard.getNumber("PathTestCruiseHeight", 9000), (int) SmartDashboard.getNumber("PathTestEndHeight", 9000),
        			SmartDashboard.getBoolean("PathTestUseBanking", true)
        	).start();
        }));
        new JoystickButton(RobotMap.leftJoy, 10).whenPressed(Commands.instant(() -> {
        	new PickupCube(
        			new Point2D.Double(SmartDashboard.getNumber("CubePickupX", 0.0), SmartDashboard.getNumber("CubePickupY", 80.0)),
        			SmartDashboard.getBoolean("CubePickupCubeForward", true),
        			SmartDashboard.getNumber("CubePickupRobotHeading", 0.0)
        	).start();
        }));
        new JoystickButton(RobotMap.leftJoy, 11).whenPressed(Commands.instant(() -> {
        	Commands.sequential(
        			new PathTo2(
                			new Point2D.Double(SmartDashboard.getNumber("PathTestX", -103.0), SmartDashboard.getNumber("PathTestY", 220.0)),
                			SmartDashboard.getNumber("PathTestHeading", 0.0),
                			SmartDashboard.getNumber("PathTestCruiseSpeed", 0.5), SmartDashboard.getNumber("PathTestEndSpeed", 0.0),
                			(int) SmartDashboard.getNumber("PathTestCruiseHeight", 9000), (int) SmartDashboard.getNumber("PathTestEndHeight", 9000),
                			SmartDashboard.getBoolean("PathTestUseBanking", true)
                	),
        			new PickupCube(
                			new Point2D.Double(SmartDashboard.getNumber("CubePickupX", 0.0), SmartDashboard.getNumber("CubePickupY", 80.0)),
                			SmartDashboard.getBoolean("CubePickupCubeForward", true),
                			SmartDashboard.getNumber("CubePickupRobotHeading", 0.0)
                	)
        	).start();
        }));*/
        
        /*new JoystickButton(rightJoy, 8).whenPressed(instantCommand(() -> {
        	driveLeftFront.setNeutralMode(NeutralMode.Brake);
        	driveLeftRear.setNeutralMode(NeutralMode.Brake);
        	driveRightFront.setNeutralMode(NeutralMode.Brake);
        	driveRightRear.setNeutralMode(NeutralMode.Brake);
        }, true));
        new JoystickButton(rightJoy, 9).whenPressed(instantCommand(() -> {
        	driveLeftFront.setNeutralMode(NeutralMode.Coast);
        	driveLeftRear.setNeutralMode(NeutralMode.Coast);
        	driveRightFront.setNeutralMode(NeutralMode.Coast);
        	driveRightRear.setNeutralMode(NeutralMode.Coast);
        }, true));
        new JoystickButton(rightJoy, 10).whenPressed(instantCommand(() -> TeleopDrive.usePID = false, true));
        new JoystickButton(rightJoy, 11).whenPressed(instantCommand(() -> TeleopDrive.usePID = true, true));*/
	}
	
	@Override
	public void robotPeriodic() {
		Scheduler.getInstance().run();
		
		SmartDashboard.putNumber("SteerLeftFront", RobotMap.steerLeftFront.getSelectedSensorPosition(0));
		SmartDashboard.putNumber("SteerLeftRear", RobotMap.steerLeftRear.getSelectedSensorPosition(0));
		SmartDashboard.putNumber("SteerRightFront", RobotMap.steerRightFront.getSelectedSensorPosition(0));
		SmartDashboard.putNumber("SteerRightRear", RobotMap.steerRightRear.getSelectedSensorPosition(0));

		SmartDashboard.putNumber("DriveLeftFront", RobotMap.driveLeftFront.getSelectedSensorPosition(0));
		SmartDashboard.putNumber("DriveLeftRear", RobotMap.driveLeftRear.getSelectedSensorPosition(0));
		SmartDashboard.putNumber("DriveRightFront", RobotMap.driveRightFront.getSelectedSensorPosition(0));
		SmartDashboard.putNumber("DriveRightRear", RobotMap.driveRightRear.getSelectedSensorPosition(0));

		/*SmartDashboard.putNumber("SpeedLeftFront", RobotMap.driveLeftFront.getSelectedSensorVelocity(0));
		SmartDashboard.putNumber("SpeedLeftRear", RobotMap.driveLeftRear.getSelectedSensorVelocity(0));
		SmartDashboard.putNumber("SpeedRightFront", RobotMap.driveRightFront.getSelectedSensorVelocity(0));
		SmartDashboard.putNumber("SpeedRightRear", RobotMap.driveRightRear.getSelectedSensorVelocity(0));*/
		
		/*SmartDashboard.putNumber("DriveLeftFrontError", RobotMap.driveLeftFront.getClosedLoopError(0));
		SmartDashboard.putNumber("DriveLeftRearError", RobotMap.driveLeftRear.getClosedLoopError(0));
		SmartDashboard.putNumber("DriveRightFrontError", RobotMap.driveRightFront.getClosedLoopError(0));
		SmartDashboard.putNumber("DriveRightRearError", RobotMap.driveRightRear.getClosedLoopError(0));*/

		SmartDashboard.putNumber("ElevatorFront", RobotMap.elevatorFront.getSelectedSensorPosition(0));
		SmartDashboard.putNumber("ElevatorRear", RobotMap.elevatorRear.getSelectedSensorPosition(0));
		
		SmartDashboard.putNumber("ElevatorFrontSpeed", RobotMap.elevatorFront.getSelectedSensorVelocity(0));
		SmartDashboard.putNumber("ElevatorOutput", RobotMap.elevatorFront.getMotorOutputPercent());

		Point2D.Double position = RobotMap.positioning.getPosition();
		SmartDashboard.putNumber("RobotX", position.x);
		SmartDashboard.putNumber("RobotY", position.y);
		SmartDashboard.putNumber("RobotHeading", RobotMap.positioning.getHeading());

		SmartDashboard.putBoolean("CubeSensorFront", !RobotMap.cubeSensorFront.get());
		SmartDashboard.putBoolean("CubeSensorRear", !RobotMap.cubeSensorRear.get());
	}
	
	@Override
	public void disabledInit() {
		RobotMap.elevator.disable();
	}
	
	@Override
	public void autonomousInit() {
		RobotMap.drive.setNeutralMode(NeutralMode.Brake);
		RobotMap.elevator.setZeroHeight();
		RobotMap.positioning.setFieldZeroHeading();
		
		autonomousCommand = RobotMap.autonomous.generateAutonomous();
		autonomousCommand.start();
	}
	
	@Override
	public void autonomousPeriodic() {
		
	}
	
	@Override
	public void teleopInit() {
		RobotMap.drive.setNeutralMode(NeutralMode.Coast);
		
		if (autonomousCommand != null) {
			autonomousCommand.cancel();
			autonomousCommand = null;
		}
	}
	
	@Override
	public void teleopPeriodic() {
		
	}
}

