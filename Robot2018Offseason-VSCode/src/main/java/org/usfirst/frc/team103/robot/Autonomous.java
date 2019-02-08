package org.usfirst.frc.team103.robot;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.command.InstantCommand;
import edu.wpi.first.wpilibj.command.PrintCommand;
import edu.wpi.first.wpilibj.command.WaitCommand;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import org.usfirst.frc.team103.command.*;
import org.usfirst.frc.team103.subsystem.Elevator;
import org.usfirst.frc.team103.util.Commands;

import java.awt.geom.Point2D;

public class Autonomous {
	public static final String START_KEY = "Starting Position",
			DEPLOY_KEY = "Deploy Action",
			DELAY_1_KEY = "Delay 1", ACTION_1_KEY = "Action 1",
			DELAY_2_KEY = "Delay 2", ACTION_2_KEY = "Action 2",
			END_KEY = "Ending Position";
	
	public static final double DRIVE_SPEED = 0.75, PICKUP_SPEED = 0.4;
	public static final double START_X = 103.0;
	public static final int SCALE_PLACE_HEIGHT = 28000;
	
	private SendableChooser<StartingPosition> startChooser;
	private SendableChooser<DeployAction> deployChooser;
	private SendableChooser<Action> action1Chooser, action2Chooser;
	private SendableChooser<EndingPosition> endChooser;
	
	private StartingPosition startPosition;
	private FieldSide switchSide, scaleSide;
	
	public void initializeOptions() {
		startChooser = new SendableChooser<>();
		fillSendableChooser(startChooser, StartingPosition.LEFT, StartingPosition.RIGHT, StartingPosition.MIDDLE);
		SmartDashboard.putData(START_KEY, startChooser);
		
		deployChooser = new SendableChooser<>();
		fillSendableChooser(deployChooser, DeployAction.SPIN_SWITCH, DeployAction.SPIN_SCALE, DeployAction.NOTHING);
		SmartDashboard.putData(DEPLOY_KEY, deployChooser);
		
		SmartDashboard.putNumber(DELAY_1_KEY, SmartDashboard.getNumber(DELAY_1_KEY, 0.0));
		
		action1Chooser = new SendableChooser<>();
		fillSendableChooser(
			action1Chooser,
			Action.NOTHING,
			Action.CROSS,
			Action.SWITCH_NEAR,
			Action.SWITCH_SIDE, Action.SWITCH_SIDE_SAME,
			Action.SWITCH_MIDDLE,
			Action.SCALE_MIDDLE,
			Action.SCALE_SIDE, Action.SCALE_SIDE_SAME,
			Action.SCALE_SWITCH_SIDE_SAME,
			Action.MAR_CHAMPS
		);
		SmartDashboard.putData(ACTION_1_KEY, action1Chooser);
		
		SmartDashboard.putNumber(DELAY_2_KEY, SmartDashboard.getNumber(DELAY_2_KEY, 0.0));
		
		action2Chooser = new SendableChooser<>();
		fillSendableChooser(
			action2Chooser,
			Action.NOTHING,
			Action.SWITCH_NEAR, Action.SWITCH_SIDE, Action.SWITCH_MIDDLE, Action.SCALE_MIDDLE, Action.SCALE_SIDE
		);
		SmartDashboard.putData(ACTION_2_KEY, action2Chooser);
		
		endChooser = new SendableChooser<>();
		fillSendableChooser(endChooser, EndingPosition.STAY, EndingPosition.SWITCH_FENCE, EndingPosition.NULL);
		SmartDashboard.putData(END_KEY, endChooser);
	}
	
	public Command generateAutonomous() {
		String message;
		while ((message = DriverStation.getInstance().getGameSpecificMessage()).isEmpty()) {
			try {
				Thread.sleep(1);
			} catch (InterruptedException e) {
				e.printStackTrace();
			}
		}
		startPosition = startChooser.getSelected();
		switchSide = FieldSide.fromCharacter(message.charAt(0));
		scaleSide = FieldSide.fromCharacter(message.charAt(1));
		System.out.println("Starting position: " + startPosition);
		System.out.println("Switch side: " + switchSide);
		System.out.println("Scale side: " + scaleSide);

		return Commands.sequential(
				Commands.instant(() -> RobotMap.positioning.setPosition(startPosition.multiplier * START_X, 15.0)),
				new WaitCommand(SmartDashboard.getNumber(DELAY_1_KEY, 0.0)),
				new PrintCommand("Delay 1 done"),
				generateDeploy(deployChooser.getSelected()),
				new PrintCommand("Deploy done"),
				generateAction1(action1Chooser.getSelected()),
				new PrintCommand("Action 1 done"),
				new WaitCommand(SmartDashboard.getNumber(DELAY_2_KEY, 0.0)),
				new PrintCommand("Delay 2 done"),
				generateAction2(action2Chooser.getSelected()),
				new PrintCommand("Action 2 done"),
				generateEnd(endChooser.getSelected()),
				new PrintCommand("End done"),
				Commands.instant(() -> RobotMap.drive.disable())
		);
	}
	
	@SafeVarargs
	private static <T> void fillSendableChooser(SendableChooser<T> chooser, T defaultOption, T... options) {
		chooser.addDefault(defaultOption.toString(), defaultOption);
		for (T option : options) {
			chooser.addObject(option.toString(), option);
		}
	}
	
	private Command generateDeploy(DeployAction deploy) {
		switch (deploy) {
		case NOTHING:
			return new InstantCommand();
		case SPIN_SWITCH:
			//return new DriveTo((startPosition.multiplier == 0.0 ? 0.5 * switchSide.multiplier : startPosition.multiplier) * START_X, 80.0, 180.0, 1.0, 0.0, 30.0, 20.0);
			return new DriveTo2(new Point2D.Double((startPosition.multiplier == 0.0 ? 0.5 * switchSide.multiplier : startPosition.multiplier) * START_X, 50.0), 180.0, 1.0, 0.1);
		case SPIN_SCALE:
			return new DriveTo2(new Point2D.Double((startPosition.multiplier == 0.0 ? 0.5 * scaleSide.multiplier : startPosition.multiplier) * START_X, 50.0), 180.0, 1.0, 0.1);
		default:
			return null;
		}
	}
	
	private Command generateAction1(Action action) {
		System.out.println("Generating " + action);
		switch (action) {
		case NOTHING:
			return new InstantCommand();
		case CROSS:
			//return new DriveTo((startPosition.multiplier == 0.0 ? switchSide.multiplier : startPosition.multiplier) * 100.0, 110.0, 0.0, DRIVE_SPEED, 0.0);
			return new DriveTo2(new Point2D.Double((startPosition.multiplier == 0.0 ? switchSide.multiplier : startPosition.multiplier) * 100.0, 110.0), 0.0, DRIVE_SPEED, 0.0);
		case SWITCH_NEAR:
			return Commands.sequential(
				new PathTo2(new Point2D.Double(switchSide.multiplier * 65.0, 125.0), 0.0, DRIVE_SPEED, 0.0, 9000, 9000, false),
				new PlaceCube(9000, 0.0, 1.0)
			);
		case SWITCH_SIDE:
			return Commands.sequential(
				//new PathTo(switchSide.multiplier * 102.0, 170.0, 90.0, DRIVE_SPEED, 0.0),
				new PathTo2(new Point2D.Double(switchSide.multiplier * 102.0, 170.0), 90.0, DRIVE_SPEED, 0.0, 9000, 9000, true),
				new PlaceCube(9000, 180.0 + switchSide.multiplier * 90.0, 1.0)
			);
		case SWITCH_SIDE_SAME:
			if (switchSide.multiplier == startPosition.multiplier) {
				return Commands.sequential(
					new PathTo2(new Point2D.Double(switchSide.multiplier * 102.0, 170.0), 90.0, DRIVE_SPEED, 0.0, 9000, 9000, true),
					new PlaceCube(9000, 180.0 + switchSide.multiplier * 90.0, 1.0)
				);
			} else {
				return new PathTo2(new Point2D.Double(startPosition.multiplier * 102.0, 170.0), 90.0, DRIVE_SPEED, 0.0, 9000, 9000, true);
			}
		case SWITCH_MIDDLE:
			return Commands.sequential(
				//new PathTo(switchSide.multiplier * 70.0, 225.0, 0.0, DRIVE_SPEED, 0.0),
				new PathTo2(new Point2D.Double(switchSide.multiplier * 65.0, 226.0), 0.0, DRIVE_SPEED, 0.0, 9000, 9000, true),
				new PlaceCube(9000, 180.0, 1.0)
			);
		case SCALE_MIDDLE:
			return Commands.sequential(
				//new PathTo(scaleSide.multiplier * 85.0, 265.0, 0.0, DRIVE_SPEED, 0.0),
				new PathTo2(new Point2D.Double(scaleSide.multiplier * 85.0, 265.0), 0.0, DRIVE_SPEED, 0.0, 9000, 9000, true),
				new PlaceCube(SCALE_PLACE_HEIGHT, 0.0, 1.0, true)
			);
		case SCALE_SIDE:
			return Commands.sequential(
				//new PathTo(scaleSide.multiplier * 121.0, 320.0, 90.0, DRIVE_SPEED, 0.0),
				new PathTo2(new Point2D.Double(scaleSide.multiplier * 121.0, 320.0), 90.0, DRIVE_SPEED, 0.0, 9000, 9000, true),
				new PlaceCube(SCALE_PLACE_HEIGHT, 180.0 + scaleSide.multiplier * 90.0, 1.0, true)
			);
		case SCALE_SIDE_SAME:
			if (scaleSide.multiplier == startPosition.multiplier) {
				return Commands.sequential(
					new PathTo2(new Point2D.Double(scaleSide.multiplier * 121.0, 320.0), 90.0, DRIVE_SPEED, 0.0, 9000, 9000, true),
					new PlaceCube(SCALE_PLACE_HEIGHT, 180.0 + scaleSide.multiplier * 90.0, 1.0, true)
				);
			} else {
				return new PathTo2(new Point2D.Double(startPosition.multiplier * 102.0, 170.0), 90.0, DRIVE_SPEED, 0.0, 9000, 9000, true);
			}
		case SCALE_SWITCH_SIDE_SAME:
			if (scaleSide.multiplier == startPosition.multiplier) {
				return Commands.sequential(
					new PathTo2(new Point2D.Double(scaleSide.multiplier * 121.0, 320.0), 90.0, DRIVE_SPEED, 0.0, 9000, 9000, true),
					new PlaceCube(SCALE_PLACE_HEIGHT, 180.0 + scaleSide.multiplier * 90.0, 1.0, true)
				);
			} else if (switchSide.multiplier == startPosition.multiplier) {
				return Commands.sequential(
					new PathTo2(new Point2D.Double(switchSide.multiplier * 102.0, 170.0), 90.0, DRIVE_SPEED, 0.0, 9000, 9000, true),
					new PlaceCube(9000, 180.0 + switchSide.multiplier * 90.0, 1.0)
				);
			} else {
				return new PathTo2(new Point2D.Double(startPosition.multiplier * 102.0, 170.0), 90.0, DRIVE_SPEED, 0.0, 9000, 9000, true);
			}
		case MAR_CHAMPS:
			if (scaleSide.multiplier == startPosition.multiplier) {
				return Commands.sequential(
					new PathTo2(new Point2D.Double(scaleSide.multiplier * 121.0, 320.0), 90.0, DRIVE_SPEED, 0.0, 9000, 9000, true),
					new PlaceCube(SCALE_PLACE_HEIGHT, 180.0 + scaleSide.multiplier * 90.0, 1.0, true),
					new PlaceCube(3000, 0.0, 0.0)
				);
			} else if (switchSide.multiplier == startPosition.multiplier) {
				return Commands.sequential(
					new PathTo2(new Point2D.Double(switchSide.multiplier * 102.0, 170.0), 90.0, DRIVE_SPEED, 0.0, 9000, 9000, true),
					new PlaceCube(9000, 180.0 + switchSide.multiplier * 90.0, 1.0),
					new PathTo2(new Point2D.Double(switchSide.multiplier * 70.25 + 20.0, 240.0), 180.0, PICKUP_SPEED, 0.1, 9000, 3000, false),
					new PickupCube(new Point2D.Double(switchSide.multiplier * 70.25, 202.5), false, 180.0),
					new PathTo2(new Point2D.Double(switchSide.multiplier * 65.0, 216.0), 180.0, PICKUP_SPEED, 0.0, 9000, 9000, false),
					new PlaceCube(9000, 180.0, 1.0)
				);
			} else {
				//return new PathTo2(new Point2D.Double(0.0, 240.0), 0.0, DRIVE_SPEED, 90.0, 9000, 9000, true);
				return new PathTo2(new Point2D.Double(startPosition.multiplier * 102.0, 170.0), 90.0, DRIVE_SPEED, 0.0, 9000, 9000, true);
			}
		default:
			return null;
		}
	}
	
	private Command generateAction2(Action action) {
		System.out.println("Generating " + action);
		switch (action) {
		case NOTHING:
			return new InstantCommand();
		case SWITCH_NEAR:
			return Commands.sequential(
				new PathTo2(new Point2D.Double(20.0, 70.0), 0.0, PICKUP_SPEED, PICKUP_SPEED, 9000, 3000, false),
				new DriveTo2(new Point2D.Double(-20.0, 70.0), 0.0, PICKUP_SPEED, 0.1),
				new PickupCube(new Point2D.Double(0.0, 107.5), true, 0.0),
				new PathTo2(new Point2D.Double(switchSide.multiplier * 65.0, 125.0), 0.0, PICKUP_SPEED, 0.0, 9000, 9000, false),
				new PlaceCube(9000, 0.0, 1.0)
			);
		case SWITCH_SIDE:
			return Commands.sequential(
				new PathTo2(new Point2D.Double(switchSide.multiplier * 70.25 + 20.0, 240.0), 180.0, PICKUP_SPEED, 0.1, 9000, 3000, false),
				new PickupCube(new Point2D.Double(switchSide.multiplier * 70.25, 202.5), false, 180.0),
				new PathTo2(new Point2D.Double(switchSide.multiplier * 102.0, 170.0), 180.0 + switchSide.multiplier * 90.0, PICKUP_SPEED, 0.0, 9000, 9000, false),
				new PlaceCube(9000, 180.0 + switchSide.multiplier * 90.0, 1.0)
			);
		case SWITCH_MIDDLE:
			return Commands.sequential(
				new PathTo2(new Point2D.Double(switchSide.multiplier * 70.25 + 20.0, 240.0), 180.0, PICKUP_SPEED, 0.1, 9000, 3000, false),
				new PickupCube(new Point2D.Double(switchSide.multiplier * 70.25, 202.5), false, 180.0),
				new PathTo2(new Point2D.Double(switchSide.multiplier * 65.0, 226.0), 180.0, PICKUP_SPEED, 0.0, 9000, 9000, false),
				new PlaceCube(9000, 180.0, 1.0)
			);
		case SCALE_MIDDLE:
			return Commands.sequential(
				new PathTo2(new Point2D.Double(scaleSide.multiplier * 70.25 + 20.0, 240.0), 180.0, PICKUP_SPEED, 0.1, 9000, 3000, false),
				new PickupCube(new Point2D.Double(scaleSide.multiplier * 70.25, 202.5), false, 180.0),
				new PathTo2(new Point2D.Double(scaleSide.multiplier * 85.0, 265.0), 0.0, PICKUP_SPEED, 0.0, 9000, 9000, false),
				new PlaceCube(SCALE_PLACE_HEIGHT, 0.0, 1.0, true)
			);
		case SCALE_SIDE:
			return Commands.sequential(
				new PathTo2(new Point2D.Double(scaleSide.multiplier * 70.25 + 20.0, 240.0), 180.0, PICKUP_SPEED, 0.1, 9000, 3000, false),
				new PickupCube(new Point2D.Double(scaleSide.multiplier * 70.25, 202.5), false, 180.0),
				new PathTo2(new Point2D.Double(scaleSide.multiplier * 121.0, 320.0), 180.0 + scaleSide.multiplier * 90.0, PICKUP_SPEED, 0.0, 9000, 9000, false),
				new PlaceCube(SCALE_PLACE_HEIGHT, 180.0 + scaleSide.multiplier * 90.0, 1.0, true)
			);
		default:
			return null;
		}
	}
	
	private Command generateEnd(EndingPosition end) {
		switch (end) {
		case STAY:
			return new InstantCommand();
		case SWITCH_FENCE:
			//return new PathTo(switchSide.multiplier * 70.0, 225.0, 0.0, DRIVE_SPEED, 0.0);
			return new PathTo2(new Point2D.Double(scaleSide.multiplier * 70.0, 226.0), 0.0, DRIVE_SPEED, 0.0, 9000, 2000);
		case NULL:
			//return new PathTo(scaleSide.multiplier * 121.0, 320.0, 0.0, DRIVE_SPEED, 0.0);
			return new PathTo2(new Point2D.Double(scaleSide.multiplier * 121.0, 320.0), 0.0, DRIVE_SPEED, 0.0, 9000, 2000);
		default:
			return null;
		}
	}
	
	private enum StartingPosition {
		LEFT("Left", -1.0), RIGHT("Right", 1.0), MIDDLE("Middle", 0.0);
		
		public final String label;
		public final double multiplier;
		StartingPosition(String label, double multiplier) {
			this.label = label;
			this.multiplier = multiplier;
		}
		@Override
		public String toString() {
			return label;
		}
	}
	
	private enum DeployAction {
		NOTHING("Nothing"), SPIN_SWITCH("Spin to Switch"), SPIN_SCALE("Spin to Scale");
		
		public final String label;
		DeployAction(String label) {
			this.label = label;
		}
		@Override
		public String toString() {
			return label;
		}
	}
	
	private enum EndingPosition {
		STAY("Stay in Place"), SWITCH_FENCE("Switch Fence"), NULL("Null Territory");
		
		public final String label;
		EndingPosition(String label) {
			this.label = label;
		}
		@Override
		public String toString() {
			return label;
		}
	}
	
	private enum Action {
		NOTHING("Nothing"),
		CROSS("Cross Line"),
		SWITCH_NEAR("Switch from Near"),
		SWITCH_SIDE("Switch from Side"), SWITCH_SIDE_SAME("Switch from Side (Same)"),
		SWITCH_MIDDLE("Switch from Middle"),
		SCALE_MIDDLE("Scale from Middle"),
		SCALE_SIDE("Scale from Side"), SCALE_SIDE_SAME("Scale from Side (Same)"),
		SCALE_SWITCH_SIDE_SAME("Scale/Switch from Side (Same)"),
		MAR_CHAMPS("MAR Champs");

		public final String label;
		Action(String label) {
			this.label = label;
		}
		@Override
		public String toString() {
			return label;
		}
	}
	
	private enum FieldSide {
		LEFT(-1.0), RIGHT(1.0);
		
		public final double multiplier;
		FieldSide(double multiplier) {
			this.multiplier = multiplier;
		}
		
		public static FieldSide fromCharacter(char c) {
			switch (c) {
			case 'L':
				return LEFT;
			case 'R':
				return RIGHT;
			default:
				return null;
			}
		}
	}
}
