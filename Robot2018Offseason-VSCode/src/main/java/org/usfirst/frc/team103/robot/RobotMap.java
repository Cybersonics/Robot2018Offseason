package org.usfirst.frc.team103.robot;

import com.ctre.phoenix.ParamEnum;
import com.ctre.phoenix.motorcontrol.*;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.XboxController;
import org.usfirst.frc.team103.subsystem.CubeHandler;
import org.usfirst.frc.team103.subsystem.Drive;
import org.usfirst.frc.team103.subsystem.Elevator;

public class RobotMap {
    public static TalonSRX driveLeftFront;
    public static TalonSRX driveLeftRear;
    public static TalonSRX driveRightFront;
    public static TalonSRX driveRightRear;
    // For steering motors, positive throttle corresponds to clockwise rotation, with
    // encoder position 0 is for straight forward
    public static TalonSRX steerLeftFront;
    public static TalonSRX steerLeftRear;
    public static TalonSRX steerRightFront;
    public static TalonSRX steerRightRear;
    public static Drive drive;

    public static Joystick leftJoy;
    public static Joystick rightJoy;
    public static XboxController controller;

    private static final double DRIVE_P = 7.5, DRIVE_I = 0.0, DRIVE_D = 75.0, DRIVE_F = 1.7, DRIVE_RAMP_RATE = 0.2;
    private static final int DRIVE_I_ZONE = 0, DRIVE_ALLOWABLE_ERROR = 0, DRIVE_MEASUREMENT_WINDOW = 1;
    private static final VelocityMeasPeriod DRIVE_MEASUREMENT_PERIOD = VelocityMeasPeriod.Period_20Ms;
    private static final double STEER_P = 10.0, STEER_I = 0.02, STEER_D = 0.0;
    private static final int STATUS_FRAME_PERIOD = 5;

    public static AHRS navX;
    public static Positioning positioning;
    public static UltrasonicPositioning ultrasonicPositioning;

    public static TalonSRX elevatorFront, elevatorRear;
    public static Servo climberClutch;
    public static Elevator elevator;

	public static TalonSRX armFront, armRear, conveyor;
	public static DigitalInput cubeSensorFront, cubeSensorRear;
	public static CubeHandler cubeHandler;

    public static Autonomous autonomous;

	public static void init() {
        driveLeftFront = new TalonSRX(10);
        driveLeftFront.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder, 0, 0);
        driveLeftFront.config_kP(0, DRIVE_P, 0);
        driveLeftFront.config_kI(0, DRIVE_I, 0);
        driveLeftFront.config_kD(0, DRIVE_D, 0);
        driveLeftFront.config_IntegralZone(0, DRIVE_I_ZONE, 0);
        driveLeftFront.config_kF(0, DRIVE_F, 0);
        driveLeftFront.configAllowableClosedloopError(0, DRIVE_ALLOWABLE_ERROR, 0);
        driveLeftFront.configClosedloopRamp(DRIVE_RAMP_RATE, 0);
        driveLeftFront.configVelocityMeasurementPeriod(DRIVE_MEASUREMENT_PERIOD, 0);
        driveLeftFront.configVelocityMeasurementWindow(DRIVE_MEASUREMENT_WINDOW, 0);
        driveLeftFront.setStatusFramePeriod(StatusFrameEnhanced.Status_2_Feedback0, STATUS_FRAME_PERIOD, 0);

        driveLeftRear = new TalonSRX(11);
        driveLeftRear.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder, 0, 0);
        driveLeftRear.config_kP(0, DRIVE_P, 0);
        driveLeftRear.config_kI(0, DRIVE_I, 0);
        driveLeftRear.config_kD(0, DRIVE_D, 0);
        driveLeftRear.config_IntegralZone(0, DRIVE_I_ZONE, 0);
        driveLeftRear.config_kF(0, DRIVE_F, 0);
        driveLeftRear.configAllowableClosedloopError(0, DRIVE_ALLOWABLE_ERROR, 0);
        driveLeftRear.configClosedloopRamp(DRIVE_RAMP_RATE, 0);
        driveLeftRear.configVelocityMeasurementPeriod(DRIVE_MEASUREMENT_PERIOD, 0);
        driveLeftRear.configVelocityMeasurementWindow(DRIVE_MEASUREMENT_WINDOW, 0);
        driveLeftRear.setStatusFramePeriod(StatusFrameEnhanced.Status_2_Feedback0, STATUS_FRAME_PERIOD, 0);

        driveRightFront = new TalonSRX(12);
        driveRightFront.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder, 0, 0);
		//XXX: REMOVE THIS BEFORE COMPETITION, SENSOR PHASE SHOULD NOT BE INVERTED
        //driveRightFront.setSensorPhase(true);
        driveRightFront.config_kP(0, DRIVE_P, 0);
        driveRightFront.config_kI(0, DRIVE_I, 0);
        driveRightFront.config_kD(0, DRIVE_D, 0);
        driveRightFront.config_IntegralZone(0, DRIVE_I_ZONE, 0);
        driveRightFront.config_kF(0, DRIVE_F, 0);
        driveRightFront.configAllowableClosedloopError(0, DRIVE_ALLOWABLE_ERROR, 0);
        driveRightFront.configClosedloopRamp(DRIVE_RAMP_RATE, 0);
        driveRightFront.configVelocityMeasurementPeriod(DRIVE_MEASUREMENT_PERIOD, 0);
        driveRightFront.configVelocityMeasurementWindow(DRIVE_MEASUREMENT_WINDOW, 0);
        driveRightFront.setStatusFramePeriod(StatusFrameEnhanced.Status_2_Feedback0, STATUS_FRAME_PERIOD, 0);

        driveRightRear = new TalonSRX(13);
        driveRightRear.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder, 0, 0);
		//XXX: REMOVE THIS BEFORE COMPETITION, SENSOR PHASE SHOULD NOT BE INVERTED
        //driveRightRear.setSensorPhase(true);
        driveRightRear.config_kP(0, DRIVE_P, 0);
        driveRightRear.config_kI(0, DRIVE_I, 0);
        driveRightRear.config_kD(0, DRIVE_D, 0);
        driveRightRear.config_IntegralZone(0, DRIVE_I_ZONE, 0);
        driveRightRear.config_kF(0, DRIVE_F, 0);
        driveRightRear.configAllowableClosedloopError(0, DRIVE_ALLOWABLE_ERROR, 0);
        driveRightRear.configClosedloopRamp(DRIVE_RAMP_RATE, 0);
        driveRightRear.configVelocityMeasurementPeriod(DRIVE_MEASUREMENT_PERIOD, 0);
        driveRightRear.configVelocityMeasurementWindow(DRIVE_MEASUREMENT_WINDOW, 0);
        driveRightRear.setStatusFramePeriod(StatusFrameEnhanced.Status_2_Feedback0, STATUS_FRAME_PERIOD, 0);

        steerLeftFront = new TalonSRX(16);
        steerLeftFront.configSelectedFeedbackSensor(FeedbackDevice.Analog, 0, 0);
        steerLeftFront.setInverted(true);
        steerLeftFront.config_kP(0, STEER_P, 0);
        steerLeftFront.config_kI(0, STEER_I, 0);
        steerLeftFront.config_kD(0, STEER_D, 0);
        steerLeftFront.config_IntegralZone(0, 100, 0);
        steerLeftFront.configAllowableClosedloopError(0, 5, 0);
        steerLeftFront.setNeutralMode(NeutralMode.Brake);
        steerLeftFront.setStatusFramePeriod(StatusFrameEnhanced.Status_2_Feedback0, STATUS_FRAME_PERIOD, 0);

        steerLeftRear = new TalonSRX(17);
        steerLeftRear.configSelectedFeedbackSensor(FeedbackDevice.Analog, 0, 0);
        steerLeftRear.setInverted(true);
        steerLeftRear.config_kP(0, STEER_P, 0);
        steerLeftRear.config_kI(0, STEER_I, 0);
        steerLeftRear.config_kD(0, STEER_D, 0);
        steerLeftRear.config_IntegralZone(0, 100, 0);
        steerLeftRear.configAllowableClosedloopError(0, 5, 0);
        steerLeftRear.setNeutralMode(NeutralMode.Brake);
        steerLeftRear.setStatusFramePeriod(StatusFrameEnhanced.Status_2_Feedback0, STATUS_FRAME_PERIOD, 0);

        steerRightFront = new TalonSRX(18);
        steerRightFront.configSelectedFeedbackSensor(FeedbackDevice.Analog, 0, 0);
        steerRightFront.setInverted(true);
        steerRightFront.config_kP(0, STEER_P, 0);
        steerRightFront.config_kI(0, STEER_I, 0);
        steerRightFront.config_kD(0, STEER_D, 0);
        steerRightFront.config_IntegralZone(0, 100, 0);
        steerRightFront.configAllowableClosedloopError(0, 5, 0);
        steerRightFront.setNeutralMode(NeutralMode.Brake);
        steerRightFront.setStatusFramePeriod(StatusFrameEnhanced.Status_2_Feedback0, STATUS_FRAME_PERIOD, 0);

        steerRightRear = new TalonSRX(19);
        steerRightRear.configSelectedFeedbackSensor(FeedbackDevice.Analog, 0, 0);
        steerRightRear.setInverted(true);
        steerRightRear.config_kP(0, STEER_P, 0);
        steerRightRear.config_kI(0, STEER_I, 0);
        steerRightRear.config_kD(0, STEER_D, 0);
        steerRightRear.config_IntegralZone(0, 100, 0);
        steerRightRear.configAllowableClosedloopError(0, 5, 0);
        steerRightRear.setNeutralMode(NeutralMode.Brake);
        steerRightRear.setStatusFramePeriod(StatusFrameEnhanced.Status_2_Feedback0, STATUS_FRAME_PERIOD, 0);

        drive = new Drive();

		elevatorFront = new TalonSRX(20);
		elevatorFront.setInverted(true);
		//XXX: REMOVE THIS BEFORE COMPETITION, SENSOR PHASE SHOULD NOT BE INVERTED
		//elevatorFront.setSensorPhase(true);
		elevatorFront.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Absolute, 0, 0);
		elevatorFront.setNeutralMode(NeutralMode.Brake);
        elevatorFront.config_kP(0, 1.0, 0);
        elevatorFront.config_kI(0, 0.0, 0);
        elevatorFront.config_kD(0, 0.0, 0);
        elevatorFront.config_kF(0, 1.2, 0);
        elevatorFront.configVelocityMeasurementWindow(1, 0);
        elevatorFront.configVelocityMeasurementPeriod(VelocityMeasPeriod.Period_10Ms, 0);
		elevatorFront.configMotionAcceleration(2400, 0);
		elevatorFront.configMotionCruiseVelocity(1200, 0);
		elevatorFront.configForwardSoftLimitThreshold(Elevator.MAX_HEIGHT, 0);
		elevatorFront.configReverseSoftLimitThreshold(Elevator.MIN_HEIGHT, 0);
		elevatorFront.configForwardSoftLimitEnable(true, 0);
		elevatorFront.configReverseSoftLimitEnable(true, 0);
		elevatorFront.configForwardLimitSwitchSource(LimitSwitchSource.FeedbackConnector, LimitSwitchNormal.NormallyOpen, 0);
		elevatorFront.configReverseLimitSwitchSource(LimitSwitchSource.FeedbackConnector, LimitSwitchNormal.NormallyOpen, 0);
		elevatorFront.configSetParameter(ParamEnum.eClearPositionOnLimitF, 0, 0, 0, 0);
		elevatorFront.configSetParameter(ParamEnum.eClearPositionOnLimitR, 1, 0, 0, 0);

		elevatorRear = new TalonSRX(21);
		elevatorRear.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Absolute, 0, 0);
		elevatorRear.setNeutralMode(NeutralMode.Brake);
		elevatorRear.follow(elevatorFront);
		
		climberClutch = new Servo(0);

		elevator = new Elevator();

		armFront = new TalonSRX(24);
		armFront.setNeutralMode(NeutralMode.Brake);
		armRear = new TalonSRX(23);
		armRear.setNeutralMode(NeutralMode.Brake);
		conveyor = new TalonSRX(22);
		conveyor.setNeutralMode(NeutralMode.Brake);
		
		cubeSensorFront = new DigitalInput(10);
		cubeSensorRear = new DigitalInput(9);

		cubeHandler = new CubeHandler();

        leftJoy = new Joystick(0);
        rightJoy = new Joystick(1);
        controller = new XboxController(2);

		navX = new AHRS(SPI.Port.kMXP);

		positioning = new Positioning();
		ultrasonicPositioning = new UltrasonicPositioning();

		autonomous = new Autonomous();
		autonomous.initializeOptions();

		//XXX: Remove before competition?
		//CameraServer.getInstance().startAutomaticCapture();
	}
}
