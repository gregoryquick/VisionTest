package org.usfirst.frc.team948.utilities;

import java.util.Timer;
import java.util.TimerTask;

import org.usfirst.frc.team948.robot.Robot;
import org.usfirst.frc.team948.robot.RobotMap;

public class PositionTracker {
//	private static final double percentForSignificance = 0.75;
//	private static final double objectDetectionDistance = 3.0; // inches
//	private static final int updatesBack = 10;
	private double x, y;
	private double prevLeftEncoder, prevRightEncoder;
//	private double[] ultraSonicReadouts = new double[updatesBack];
//	private int index = 0;
	private Timer timer;

	public void init(double x, double y) {
		this.x = x;
		this.y = y;
		prevLeftEncoder = Robot.robotMap.leftEncoder.get();
		prevRightEncoder = Robot.robotMap.rightEncoder.get();
	}

	public synchronized void updatePosition() {
		double leftEncoder = Robot.robotMap.leftEncoder.get();
		double rightEncoder = Robot.robotMap.rightEncoder.get();
		double leftDelta = leftEncoder - prevLeftEncoder;
		double rightDelta = rightEncoder - prevRightEncoder;
//		 double distance = ((Math.abs(leftDelta) > Math.abs(rightDelta)) ?
		// leftDelta : rightDelta)
		// / Robot.drive.getTicksPerInch();
		double distance = (leftDelta + rightDelta) / 2 / Robot.drive.getTicksPerInch();
		double heading = Math.toRadians(RobotMap.continuousGyro.getAngle());
		x += distance * Math.sin(heading);
		y += distance * Math.cos(heading);
		prevLeftEncoder = leftEncoder;
		prevRightEncoder = rightEncoder;
//		ultraSonicReadouts[index] = RobotMap.ultraSound.getVoltage();
//		index++;
//		index %= updatesBack;
	}

	public double getX() {
		return x;
	}

	public double getY() {
		return y;
	}
	
//	public boolean objectInfront(boolean useAverageing) {
//		if(useAverageing){
//			double agglutinatedNumber = 0.0;
//			for (double a : ultraSonicReadouts) {
//				agglutinatedNumber += RobotMap.ultraSound.getDistanceInches(a) <= objectDetectionDistance ? 1.0 : 0.0;
//			}
//			return agglutinatedNumber / updatesBack >= percentForSignificance;
//		}else{
//			return RobotMap.ultraSound.getDistanceInches(ultraSonicReadouts[updatesBack - 1]) <= objectDetectionDistance;
//		}
//	}

	public void start() {
		System.out.println("Started Position Tracker");
		timer = new Timer();
		timer.schedule(new TimerTask() {

			@Override
			public void run() {
//				System.out.println("This is spam from a timer at " + System.currentTimeMillis());
				updatePosition();
			}

		}, 0, 50);
	}
	
	public void stop(){
		if(timer != null)
			timer.cancel();
	}

	@Override
	public String toString() {
		return "X cord:" +x + "| Y Cord: " + y;

	}

	public synchronized void reset() {
		prevLeftEncoder = 0;
		prevRightEncoder = 0;
		x = 0;
		y = 0;
	}
}
