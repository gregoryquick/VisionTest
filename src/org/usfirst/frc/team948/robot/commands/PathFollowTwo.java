package org.usfirst.frc.team948.robot.commands;

import org.usfirst.frc.team948.robot.OI;
import org.usfirst.frc.team948.robot.OI;
import org.usfirst.frc.team948.robot.Robot;
import org.usfirst.frc.team948.robot.RobotMap;
import org.usfirst.frc.team948.robot.commands.DriveStraitToFieldPosition.segmentType;
import org.usfirst.frc.team948.utilities.FalconPathPlanner;
import org.usfirst.frc.team948.utilities.MathUtil;
import org.usfirst.frc.team948.utilities.Point2D;
import org.usfirst.frc.team948.utilities.MathUtil;

import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.command.CommandGroup;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class PathFollowTwo extends Command {
	//I am aware that the number of chords is actual the number of waypoints minus one
	private double numOfChords;
	private final double fakeTime = 5.0;
	private double fakeTimeStep;
	private final double whealSeperationInches = 27.0;
	private Point2D[] pathNodes = {new Point2D(0.0,0.0)};
	private Double previosHeadingDelta;
	private final Point2D endingLocation;
//	private final double tempSpeedNumber = 0.7;
	private int index = 0;
	private int maxIndex = 0;
	private final double headingAtEnd;
	private double minDistance = 1000000000000000000000.0;
	
	public PathFollowTwo(Point2D end, double finalHeading){
		endingLocation = end;
		headingAtEnd = finalHeading;
		requires(Robot.drive);
	}

	@Override
	protected void initialize() {
		double[] startPose = new double[3];
		startPose[0] = Robot.positionTracker.getX();
		startPose[1] = Robot.positionTracker.getY();
		startPose[2] = RobotMap.continuousGyro.getAngle();
		numOfChords = distance(new Point2D(startPose[0],startPose[1]),endingLocation)/2.0;
		numOfChords = numOfChords <= 1.0 ? 1.0 : numOfChords;
		fakeTimeStep = fakeTime/numOfChords;
		double[][] waypoints = {
				{startPose[0],startPose[1]},
//				{(startPose[0]+endingLocation.x)/2.0,(startPose[1]+endingLocation.y)/2.0},
				{endingLocation.x,endingLocation.y}
		};
		double[] headings = {startPose[2],headingAtEnd};
//		headings[1] = (headings[1]%360.0) -180.0;
		FalconPathPlanner planner;
		try {
			planner = new FalconPathPlanner(waypoints,headings);
			planner.calculate(fakeTime, fakeTimeStep, whealSeperationInches);
			double[][] tempPathNodes = planner.smoothPath;
//			double[][] tempPathNodes = planner.nodeOnlyPath;
			pathNodes = new Point2D[tempPathNodes.length];
			for(int i = 0; i < tempPathNodes.length;i++){
				pathNodes[i] = new Point2D(tempPathNodes[i][0],tempPathNodes[i][1]);
			}
			maxIndex = pathNodes.length - 1;
		} catch (Exception e) {
			end();
			e.printStackTrace();
			return;
		}
//		Robot.drive.driveOnHeadingEnd(false);
		Robot.drive.driveOnHeadingInit(RobotMap.continuousGyro.getAngle());
	}
	
	private double distance(Point2D pointOne, Point2D pointTwo){
		double ysquared = (pointOne.y-pointTwo.y)*(pointOne.y-pointTwo.y);
		double xsquared = (pointOne.x-pointTwo.x)*(pointOne.x-pointTwo.x);
		return Math.sqrt(ysquared + xsquared);
	}
	
	private double getHeading(Point2D pointOne, Point2D pointTwo){
		return Math.toDegrees(Math.atan2(pointTwo.x - pointOne.x, pointTwo.y - pointOne.y));
	}

	@Override
	protected void execute() {
		if (index < pathNodes.length) {
			Point2D currentLocation = new Point2D(Robot.positionTracker.getX(), Robot.positionTracker.getY());
			Point2D nextTarget;
			Point2D currentTarget;
			if (index < maxIndex) {
				if (index > 0) {
					double distanceToPrev = distance(currentLocation, pathNodes[index - 1]);
					double distanceToCurrent = distance(currentLocation, pathNodes[index]);
					double distanceToNext = distance(currentLocation, pathNodes[index + 1]);
					System.out.println("distanceToPrev:" + distanceToPrev);
					System.out.println("distanceToCurrent:" + distanceToCurrent);
					System.out.println("distanceToNext:" + distanceToNext);
					if (distanceToNext < distanceToCurrent || minDistance < 0.3) {
						index++;
						minDistance = 1000000000000000.0;
					} else if (distanceToPrev < distanceToCurrent && distanceToCurrent > 5.0) {
						index--;
						minDistance = 10000000000.0;
					}
					currentTarget = pathNodes[index];
					if (index < maxIndex) {
						nextTarget = pathNodes[index + 1];
					} else {
						nextTarget = pathNodes[index];
					}
				} else {
					double distanceToCurrent = distance(currentLocation, pathNodes[index]);
					double distanceToNext = distance(currentLocation, pathNodes[index + 1]);
					System.out.println("distanceToCurrent:" + distanceToCurrent);
					System.out.println("distanceToNext:" + distanceToNext);
					if (distanceToNext < distanceToCurrent || minDistance < 0.3) {
						index++;
						minDistance = 10000000.0;
					}
					currentTarget = pathNodes[index];
					if (index < maxIndex) {
						nextTarget = pathNodes[index + 1];
					} else {
						nextTarget = pathNodes[index];
					}
				}
			} else {
				nextTarget = pathNodes[index];
				currentTarget = pathNodes[index];
			}
			double power;
			double distance_ = distance(currentLocation, currentTarget);
			if (minDistance > distance_)
				minDistance = distance_;
			double nextDistance = distance(currentLocation, nextTarget);
			double heading_ = getHeading(currentLocation, currentTarget);
			double nextHeading = getHeading(currentLocation, nextTarget);
			double lemNot = Math.tanh(nextDistance - distance_) + 1.0;
			if (maxIndex - index < 20) {
				power = MathUtil.clamp(Math.tanh(lemNot * distance_), 0.2, 0.4);
			} else if (maxIndex - index < 10) {
				power = MathUtil.clamp(Math.tanh(lemNot * distance_), 0.1, 0.3);
			} else {
				power = MathUtil.clamp(Math.tanh(lemNot * distance_), 0.4, 0.7);
			}
			double updatedHeading = (7.0 * heading_ + nextHeading) / 8.0;
			System.out.println("HeadingToCurrent:" + heading_);
			System.out.println("HeadingToNext:" + nextHeading);
			//		updatedHeading = heading_;
			//		if(nextDistance > 2.0){
			//			updatedHeading = heading_ + ((distance_/(nextDistance*3.0))*nextHeading);
			//		}else{
			//			updatedHeading = nextHeading;
			//		}
			double angleDelta = heading_ - RobotMap.continuousGyro.getAngle();
//			if (RobotMap.preferences.getBoolean("h0P7B", false))
//				angleDelta *= -1.0;
					if(Math.abs(angleDelta) > 3.0 ){
			double range = RobotMap.preferences.getDouble("k7u4Q", 15.0) * Math.exp(Math.tanh(angleDelta));
			if(previosHeadingDelta != null){
				if(Math.abs(previosHeadingDelta - angleDelta) > 4.0){
					range = 1.0;
				}
			}
			double normalizedDelta = MathUtil.clamp(angleDelta, -range, range);
			previosHeadingDelta = angleDelta;
			Robot.drive.driveOnHeading(power, RobotMap.continuousGyro.getAngle() + normalizedDelta);
					}else{
						Robot.drive.driveOnHeadingLazy(power, updatedHeading);
					}
			if (minDistance > 25.0 && Math.abs(angleDelta) > 20.0) {
//				index--;
				System.out.println("\n\n\n\nRecalulating Path\n\n\n\n");
//				previosHeadingDelta = null;
//				initialize();
//				end();
				new CommandGroup(){{
//					addSequential(new PathFollowTwo(endingLocation, headingAtEnd));
				}}.start();
//				end();
			}
		}
//		else{
//			Robot.drive.driveOnHeading(0.15, RobotMap.continuousGyro.getAngle());
//		}
	}
	
	@Override
	protected boolean isFinished() {
		if (index < pathNodes.length) {
			System.out.println("minDistance:" + minDistance);
			Point2D currentLocation = new Point2D(Robot.positionTracker.getX(), Robot.positionTracker.getY());
			double distance_ = distance(currentLocation, endingLocation);
			System.out.println("Distance:" + distance_);
			boolean conOne = minDistance < 0.7 || distance_ < 7.0;
			System.out.println(index);
			return conOne && maxIndex - index < 10; //index == maxIndex;
		}
		return false;
	}

	@Override
	protected void end() {
		Robot.drive.driveOnHeadingEnd(true);
	}

	@Override
	protected void interrupted() {
		end();
	}
}
