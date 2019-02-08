package org.usfirst.frc.team103.robot;

import static org.usfirst.frc.team103.robot.RobotMap.*;

import java.awt.geom.Point2D;

import org.opencv.core.CvType;
import org.opencv.core.Mat;
import org.opencv.core.Scalar;
import org.usfirst.frc.team103.subsystem.Drive;
import org.usfirst.frc.team103.util.Threads;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Positioning {
	private static final double TWO_PI = 2.0 * Math.PI;
	private static final double WHEEL_BASE_LENGTH = Drive.WHEEL_BASE_LENGTH, WHEEL_BASE_WIDTH = Drive.WHEEL_BASE_WIDTH;
	private static final double DIAGONAL_LENGTH = Math.hypot(WHEEL_BASE_WIDTH / 2.0, WHEEL_BASE_LENGTH / 2.0);
	private static final double DIAGONAL_ANGLE = Math.atan(WHEEL_BASE_WIDTH / WHEEL_BASE_LENGTH);
	private static final double ENCODER_SCALE = 1.0 / ((80.0 * 6.67) / (Drive.WHEEL_DIAMETER * Math.PI)), STEERING_SCALE = TWO_PI / 1024.0;
	private static final int UPDATE_PERIOD = 15;
	//private static final String[] MSE_LABELS = { "KeepAll", "DropLeftFront", "DropRightFront", "DropLeftRear", "DropRightRear" };
	private static final double OUTLIER_K = 2.0, VARIANCE_K = 1.5;

	private volatile double fieldZeroHeading;
	private volatile double prevRobotX, prevRobotY, prevRobotHeading;
	private int prevEncoderA, prevEncoderB, prevEncoderC, prevEncoderD;
	private double prevAngleA, prevAngleB, prevAngleC, prevAngleD;
	private volatile boolean externalPositionUpdate = false;
	
	private static final int IMAGE_WIDTH = 600, IMAGE_HEIGHT = 480;
	private static final int BUFFER_SIZE = 20;
	private static final double PPI = 10.0;
	private static final Scalar RED = new Scalar(0, 0, 255), GREEN = new Scalar(0, 255, 0), BLUE = new Scalar(255, 0, 0),
			WHITE = new Scalar(255, 255, 255), BLACK = new Scalar(0, 0, 0);
	//private final CvSource imageSource;
	private final Mat image = new Mat(IMAGE_HEIGHT, IMAGE_WIDTH, CvType.CV_8UC3);
	private final double[] robotXBuffer = new double[BUFFER_SIZE],
			robotYBuffer = new double[BUFFER_SIZE],
			robotHeadingBuffer = new double[BUFFER_SIZE],
			projAxBuffer = new double[BUFFER_SIZE],
			projAyBuffer = new double[BUFFER_SIZE],
			projBxBuffer = new double[BUFFER_SIZE],
			projByBuffer = new double[BUFFER_SIZE],
			projCxBuffer = new double[BUFFER_SIZE],
			projCyBuffer = new double[BUFFER_SIZE],
			projDxBuffer = new double[BUFFER_SIZE],
			projDyBuffer = new double[BUFFER_SIZE];
	private final int[] positionBuffer = new int[BUFFER_SIZE];
	private final double[] robotXBufferCopy = new double[BUFFER_SIZE],
			robotYBufferCopy = new double[BUFFER_SIZE],
			robotHeadingBufferCopy = new double[BUFFER_SIZE],
			projAxBufferCopy = new double[BUFFER_SIZE],
			projAyBufferCopy = new double[BUFFER_SIZE],
			projBxBufferCopy = new double[BUFFER_SIZE],
			projByBufferCopy = new double[BUFFER_SIZE],
			projCxBufferCopy = new double[BUFFER_SIZE],
			projCyBufferCopy = new double[BUFFER_SIZE],
			projDxBufferCopy = new double[BUFFER_SIZE],
			projDyBufferCopy = new double[BUFFER_SIZE];
	private final int[] positionBufferCopy = new int[BUFFER_SIZE];
	private int bufferIndex = 0;
	
	public Positioning() {
		//imageSource = CameraServer.getInstance().putVideo("Positioning", IMAGE_WIDTH, IMAGE_HEIGHT);
		Threads.scheduleAtFixedRate(this::update, UPDATE_PERIOD);
		/*executor.scheduleAtFixedRate(() -> {
			int index;
			double robotX, robotY;
			synchronized (Positioning.this) {
				index = bufferIndex;
				int currentIndex = (index + BUFFER_SIZE - 1) % BUFFER_SIZE;
				robotX = robotXBuffer[currentIndex];
				robotY = robotYBuffer[currentIndex];
				System.arraycopy(robotXBuffer, 0, robotXBufferCopy, 0, BUFFER_SIZE);
				System.arraycopy(robotYBuffer, 0, robotYBufferCopy, 0, BUFFER_SIZE);
				System.arraycopy(robotHeadingBuffer, 0, robotHeadingBufferCopy, 0, BUFFER_SIZE);
				System.arraycopy(projAxBuffer, 0, projAxBufferCopy, 0, BUFFER_SIZE);
				System.arraycopy(projAyBuffer, 0, projAyBufferCopy, 0, BUFFER_SIZE);
				System.arraycopy(projBxBuffer, 0, projBxBufferCopy, 0, BUFFER_SIZE);
				System.arraycopy(projByBuffer, 0, projByBufferCopy, 0, BUFFER_SIZE);
				System.arraycopy(projCxBuffer, 0, projCxBufferCopy, 0, BUFFER_SIZE);
				System.arraycopy(projCyBuffer, 0, projCyBufferCopy, 0, BUFFER_SIZE);
				System.arraycopy(projDxBuffer, 0, projDxBufferCopy, 0, BUFFER_SIZE);
				System.arraycopy(projDyBuffer, 0, projDyBufferCopy, 0, BUFFER_SIZE);
				System.arraycopy(positionBuffer, 0, positionBufferCopy, 0, BUFFER_SIZE);
			}
			Imgproc.rectangle(image, new Point(0, 0), new Point(IMAGE_WIDTH, IMAGE_HEIGHT), WHITE, -1);
			for (int offset = 0, i = index; offset < BUFFER_SIZE; offset++, i = (i + offset) % BUFFER_SIZE) {
				int pos = positionBufferCopy[i];
				double rx = (robotXBufferCopy[i] - robotX) * PPI + IMAGE_WIDTH / 2, ry = IMAGE_HEIGHT / 2 - (robotYBufferCopy[i] - robotY) * PPI;
				double pax = (projAxBufferCopy[i] - robotX) * PPI + IMAGE_WIDTH / 2, pay = IMAGE_HEIGHT / 2 - (projAyBufferCopy[i] - robotY) * PPI;
				double pbx = (projBxBufferCopy[i] - robotX) * PPI + IMAGE_WIDTH / 2, pby = IMAGE_HEIGHT / 2 - (projByBufferCopy[i] - robotY) * PPI;
				double pcx = (projCxBufferCopy[i] - robotX) * PPI + IMAGE_WIDTH / 2, pcy = IMAGE_HEIGHT / 2 - (projCyBufferCopy[i] - robotY) * PPI;
				double pdx = (projDxBufferCopy[i] - robotX) * PPI + IMAGE_WIDTH / 2, pdy = IMAGE_HEIGHT / 2 - (projDyBufferCopy[i] - robotY) * PPI;
				//double ax = (robotXBufferCopy[i] + DIAGONAL_LENGTH * Math.cos(robotHeadingBufferCopy[i] + DIAGONAL_ANGLE)) * PPI + IMAGE_WIDTH / 2;
				Point r = new Point(rx, ry);
				Point pa = new Point(pax, pay);
				Point pb = new Point(pbx, pby);
				Point pc = new Point(pcx, pcy);
				Point pd = new Point(pdx, pdy);
				Imgproc.circle(image, r, 20, pos > 0 ? RED : GREEN, 2);
				Imgproc.circle(image, pa, 10, pos == 1 ? RED : GREEN, 2);
				Imgproc.circle(image, pb, 10, pos == 2 ? RED : GREEN, 2);
				Imgproc.circle(image, pc, 10, pos == 3 ? RED : GREEN, 2);
				Imgproc.circle(image, pd, 10, pos == 4 ? RED : GREEN, 2);
				Imgproc.line(image, pa, pb, BLACK, 1);
				Imgproc.line(image, pb, pd, BLACK, 1);
				Imgproc.line(image, pd, pc, BLACK, 1);
				Imgproc.line(image, pc, pa, BLACK, 1);
			}
			imageSource.putFrame(image);
		}, 0, 100, TimeUnit.MILLISECONDS);*/
	}
	
	private long lastUpdate = 0;
	private int count = 0;
	
	private void update() {
		long time = System.currentTimeMillis();
		SmartDashboard.putNumber("UpdateTime", time - lastUpdate);
		lastUpdate = time;

		double robotHeading = -Math.toRadians((navX.getYaw() + 360.0) % 360.0 - fieldZeroHeading) + Math.PI / 2.0;
		double prevRobotXTemp, prevRobotYTemp;
		synchronized (this) {
			prevRobotXTemp = prevRobotX;
			prevRobotYTemp = prevRobotY;
			externalPositionUpdate = false;
		}

		int encoderA = driveLeftFront.getSelectedSensorPosition(0);
		int encoderB = driveRightFront.getSelectedSensorPosition(0);
		int encoderC = driveLeftRear.getSelectedSensorPosition(0);
		int encoderD = driveRightRear.getSelectedSensorPosition(0);

		double distanceA = (encoderA - prevEncoderA) * ENCODER_SCALE;
		double distanceB = (encoderB - prevEncoderB) * ENCODER_SCALE;
		double distanceC = (encoderC - prevEncoderC) * ENCODER_SCALE;
		double distanceD = (encoderD - prevEncoderD) * ENCODER_SCALE;

		/*SmartDashboard.putNumber("RobotDistanceA", distanceA);
		SmartDashboard.putNumber("RobotDistanceB", distanceB);
		SmartDashboard.putNumber("RobotDistanceC", distanceC);
		SmartDashboard.putNumber("RobotDistanceD", distanceD);*/

		prevEncoderA = encoderA;
		prevEncoderB = encoderB;
		prevEncoderC = encoderC;
		prevEncoderD = encoderD;

		double currentAngleA = ((robotHeading - steerLeftFront.getSelectedSensorPosition(0) * STEERING_SCALE) % TWO_PI + TWO_PI) % TWO_PI;
		double currentAngleB = ((robotHeading - steerRightFront.getSelectedSensorPosition(0) * STEERING_SCALE) % TWO_PI + TWO_PI) % TWO_PI;
		double currentAngleC = ((robotHeading - steerLeftRear.getSelectedSensorPosition(0) * STEERING_SCALE) % TWO_PI + TWO_PI) % TWO_PI;
		double currentAngleD = ((robotHeading - steerRightRear.getSelectedSensorPosition(0) * STEERING_SCALE) % TWO_PI + TWO_PI) % TWO_PI;

		double angleA = (prevAngleA + currentAngleA) / 2.0;
		double angleB = (prevAngleB + currentAngleB) / 2.0;
		double angleC = (prevAngleC + currentAngleC) / 2.0;
		double angleD = (prevAngleD + currentAngleD) / 2.0;
		
		/*SmartDashboard.putNumber("RobotAngleA", Math.toDegrees(angleA) % 360.0);
		SmartDashboard.putNumber("RobotAngleB", Math.toDegrees(angleB) % 360.0);
		SmartDashboard.putNumber("RobotAngleC", Math.toDegrees(angleC) % 360.0);
		SmartDashboard.putNumber("RobotAngleD", Math.toDegrees(angleD) % 360.0);*/

		prevAngleA = currentAngleA;
		prevAngleB = currentAngleB;
		prevAngleC = currentAngleC;
		prevAngleD = currentAngleD;

		double prevCosPlus = Math.cos(prevRobotHeading + DIAGONAL_ANGLE);
		double prevCosMinus = Math.cos(prevRobotHeading - DIAGONAL_ANGLE);
		double prevSinPlus = Math.sin(prevRobotHeading + DIAGONAL_ANGLE);
		double prevSinMinus = Math.sin(prevRobotHeading - DIAGONAL_ANGLE);

		double prevAx = prevRobotXTemp + DIAGONAL_LENGTH * prevCosPlus;
		double prevAy = prevRobotYTemp + DIAGONAL_LENGTH * prevSinPlus;
		double prevBx = prevRobotXTemp + DIAGONAL_LENGTH * prevCosMinus;
		double prevBy = prevRobotYTemp + DIAGONAL_LENGTH * prevSinMinus;
		double prevCx = prevRobotXTemp - DIAGONAL_LENGTH * prevCosMinus;
		double prevCy = prevRobotYTemp - DIAGONAL_LENGTH * prevSinMinus;
		double prevDx = prevRobotXTemp - DIAGONAL_LENGTH * prevCosPlus;
		double prevDy = prevRobotYTemp - DIAGONAL_LENGTH * prevSinPlus;

		double projAx = prevAx + distanceA * Math.cos(angleA);
		double projAy = prevAy + distanceA * Math.sin(angleA);
		double projBx = prevBx + distanceB * Math.cos(angleB);
		double projBy = prevBy + distanceB * Math.sin(angleB);
		double projCx = prevCx + distanceC * Math.cos(angleC);
		double projCy = prevCy + distanceC * Math.sin(angleC);
		double projDx = prevDx + distanceD * Math.cos(angleD);
		double projDy = prevDy + distanceD * Math.sin(angleD);

		double cosPlus = Math.cos(robotHeading + DIAGONAL_ANGLE);
		double cosMinus = Math.cos(robotHeading - DIAGONAL_ANGLE);
		double sinPlus = Math.sin(robotHeading + DIAGONAL_ANGLE);
		double sinMinus = Math.sin(robotHeading - DIAGONAL_ANGLE);

		double robotXAll = (projAx + projBx + projCx + projDx) / 4.0;
		double robotXDropA = (projBx + projCx + projDx + DIAGONAL_LENGTH * cosPlus) / 3.0;
		double robotXDropB = (projAx + projCx + projDx + DIAGONAL_LENGTH * cosMinus) / 3.0;
		double robotXDropC = (projAx + projBx + projDx - DIAGONAL_LENGTH * cosMinus) / 3.0;
		double robotXDropD = (projAx + projBx + projCx - DIAGONAL_LENGTH * cosPlus) / 3.0;

		double robotYAll = (projAy + projBy + projCy + projDy) / 4.0;
		double robotYDropA = (projBy + projCy + projDy + DIAGONAL_LENGTH * sinPlus) / 3.0;
		double robotYDropB = (projAy + projCy + projDy + DIAGONAL_LENGTH * sinMinus) / 3.0;
		double robotYDropC = (projAy + projBy + projDy - DIAGONAL_LENGTH * sinMinus) / 3.0;
		double robotYDropD = (projAy + projBy + projCy - DIAGONAL_LENGTH * sinPlus) / 3.0;

		double mseAll = (Math.pow(robotXAll + DIAGONAL_LENGTH * cosPlus - projAx, 2.0)
				+ Math.pow(robotYAll + DIAGONAL_LENGTH * sinPlus - projAy, 2.0)
				+ Math.pow(robotXAll + DIAGONAL_LENGTH * cosMinus - projBx, 2.0)
				+ Math.pow(robotYAll + DIAGONAL_LENGTH * sinMinus - projBy, 2.0)
				+ Math.pow(robotXAll - DIAGONAL_LENGTH * cosMinus - projCx, 2.0)
				+ Math.pow(robotYAll - DIAGONAL_LENGTH * sinMinus - projCy, 2.0)
				+ Math.pow(robotXAll - DIAGONAL_LENGTH * cosPlus - projDx, 2.0)
				+ Math.pow(robotYAll - DIAGONAL_LENGTH * sinPlus - projDy, 2.0)) / 4.0;
		double mseDropA = (Math.pow(robotXDropA + DIAGONAL_LENGTH * cosMinus - projBx, 2.0)
				+ Math.pow(robotYDropA + DIAGONAL_LENGTH * sinMinus - projBy, 2.0)
				+ Math.pow(robotXDropA - DIAGONAL_LENGTH * cosMinus - projCx, 2.0)
				+ Math.pow(robotYDropA - DIAGONAL_LENGTH * sinMinus - projCy, 2.0)
				+ Math.pow(robotXDropA - DIAGONAL_LENGTH * cosPlus - projDx, 2.0)
				+ Math.pow(robotYDropA - DIAGONAL_LENGTH * sinPlus - projDy, 2.0)) / 3.0;
		double mseDropB = (Math.pow(robotXDropB + DIAGONAL_LENGTH * cosPlus - projAx, 2.0)
				+ Math.pow(robotYDropB + DIAGONAL_LENGTH * sinPlus - projAy, 2.0)
				+ Math.pow(robotXDropB - DIAGONAL_LENGTH * cosMinus - projCx, 2.0)
				+ Math.pow(robotYDropB - DIAGONAL_LENGTH * sinMinus - projCy, 2.0)
				+ Math.pow(robotXDropB - DIAGONAL_LENGTH * cosPlus - projDx, 2.0)
				+ Math.pow(robotYDropB - DIAGONAL_LENGTH * sinPlus - projDy, 2.0)) / 3.0;
		double mseDropC = (Math.pow(robotXDropC + DIAGONAL_LENGTH * cosPlus - projAx, 2.0)
				+ Math.pow(robotYDropC + DIAGONAL_LENGTH * sinPlus - projAy, 2.0)
				+ Math.pow(robotXDropC + DIAGONAL_LENGTH * cosMinus - projBx, 2.0)
				+ Math.pow(robotYDropC + DIAGONAL_LENGTH * sinMinus - projBy, 2.0)
				+ Math.pow(robotXDropC - DIAGONAL_LENGTH * cosPlus - projDx, 2.0)
				+ Math.pow(robotYDropC - DIAGONAL_LENGTH * sinPlus - projDy, 2.0)) / 3.0;
		double mseDropD = (Math.pow(robotXDropD + DIAGONAL_LENGTH * cosPlus - projAx, 2.0)
				+ Math.pow(robotYDropD + DIAGONAL_LENGTH * sinPlus - projAy, 2.0)
				+ Math.pow(robotXDropD + DIAGONAL_LENGTH * cosMinus - projBx, 2.0)
				+ Math.pow(robotYDropD + DIAGONAL_LENGTH * sinMinus - projBy, 2.0)
				+ Math.pow(robotXDropD - DIAGONAL_LENGTH * cosMinus - projCx, 2.0)
				+ Math.pow(robotYDropD - DIAGONAL_LENGTH * sinMinus - projCy, 2.0)) / 3.0;

		/*SmartDashboard.putNumber("MSEAll", mseAll);
		SmartDashboard.putNumber("MSEDropA", mseDropA);
		SmartDashboard.putNumber("MSEDropB", mseDropB);
		SmartDashboard.putNumber("MSEDropC", mseDropC);
		SmartDashboard.putNumber("MSEDropD", mseDropD);*/

		double robotXs[] = {robotXAll, robotXDropA, robotXDropB, robotXDropC, robotXDropD};
		double robotYs[] = {robotYAll, robotYDropA, robotYDropB, robotYDropC, robotYDropD};
		double mses[] = {mseAll, mseDropA, mseDropB, mseDropC, mseDropD};

		double mseMean = 0.0;
		for (int i = 0; i < mses.length; i++) mseMean += mses[i];
		mseMean /= mses.length;
		double mseVariance = 0.0;
		for (int i = 0; i < mses.length; i++) mseVariance += Math.pow(mses[i] - mseMean, 2.0);
		mseVariance /= mses.length;

		/*SmartDashboard.putNumber("MSEMean", mseMean);
		SmartDashboard.putNumber("MSEVariance", mseVariance);*/

		int mseMinIndex = 0;
		double mseMin = Double.MAX_VALUE;
		for (int i = 0; i < mses.length; i++) {
			if (mses[i] < mseMin) {
				mseMinIndex = i;
				mseMin = mses[i];
			}
		}
		/*Arrays.sort(mses);
		int positionIndex = (mseMin < mses[1] - OUTLIER_K * (mses[3] - mses[1]) ? mseMinIndex : 0);*/
		int positionIndex = (mseMin < mseMean - VARIANCE_K * Math.sqrt(mseVariance) ? mseMinIndex : 0);
		
		/*SmartDashboard.putNumber("MSEMinimum", mseMin);
		SmartDashboard.putNumber("MSEThreshold", mses[1] - OUTLIER_K * (mses[3] - mses[1]));*/
		/*SmartDashboard.putNumber("PositionChoiceIndex", positionIndex);
		SmartDashboard.putString("PositionChoice", MSE_LABELS[positionIndex]);*/

		double robotX = robotXs[positionIndex];
		double robotY = robotYs[positionIndex];

		synchronized (this) {
			if (!externalPositionUpdate) {
				prevRobotX = robotX;
				prevRobotY = robotY;
			} else {
				prevRobotX += (robotX - prevRobotXTemp);
				prevRobotY += (robotY - prevRobotYTemp);
			}
		}
		prevRobotHeading = robotHeading;
		
		/*SmartDashboard.putNumber("RobotX", getX());
		SmartDashboard.putNumber("RobotY", getY());
		SmartDashboard.putNumber("RobotHeading", getHeading());*/
		//SmartDashboard.putNumber("RobotHeadingRaw", Math.toDegrees(prevRobotHeading) % 360.0);
		
		/*if (count++ % 2 == 0) {
			synchronized (this) {
				robotXBuffer[bufferIndex] = robotX;
				robotYBuffer[bufferIndex] = robotY;
				robotHeadingBuffer[bufferIndex] = robotHeading;
				positionBuffer[bufferIndex] = positionIndex;
				projAxBuffer[bufferIndex] = projAx;
				projAyBuffer[bufferIndex] = projAy;
				projBxBuffer[bufferIndex] = projBx;
				projByBuffer[bufferIndex] = projBy;
				projCxBuffer[bufferIndex] = projCx;
				projCyBuffer[bufferIndex] = projCy;
				projDxBuffer[bufferIndex] = projDx;
				projDyBuffer[bufferIndex] = projDy;
				bufferIndex = (bufferIndex + 1) % BUFFER_SIZE;
			}
		}*/
	}

	public void setFieldZeroHeading() {
		fieldZeroHeading = (RobotMap.navX.getYaw() + 360.0) % 360.0;
	}

	public synchronized Point2D.Double getPosition() {
		return new Point2D.Double(prevRobotX, prevRobotY);
	}
	
	public synchronized void setPosition(double x, double y) {
		prevRobotX = x;
		prevRobotY = y;
		externalPositionUpdate = true;
	}
	
	public synchronized void mergePosition(double x, double y, double weight) {
		prevRobotX = weight * x + (1.0 - weight) * prevRobotX;
		prevRobotY = weight * y + (1.0 - weight) * prevRobotY;
		externalPositionUpdate = true;
	}
	
	public double getX() {
		return prevRobotX;
	}
	
	public double getY() {
		return prevRobotY;
	}
	
	public double getHeading() {
		double heading = -Math.toDegrees(prevRobotHeading - Math.PI / 2.0) % 360.0;
		if (heading < 0) heading += 360.0;
		return heading;
	}

}
