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

public class PathFollowOne extends Command {
	//I am aware that the number of chords is actual the number of waypoints minus one
	private double numOfChords;
	private final double fakeTime = 5.0;
	private double fakeTimeStep;
	private final double whealSeperationInches = 27.0;
	private double[][] pathNodes;
	private double[] startPose;
	private Point2D endingLocation;
	private final boolean dynamicChordNumber;
	private final double tempSpeedNumber = 0.7;
	private double chordTime;
	public  PathFollowOne(Point2D end,double numberOfWaypoints,double chordTime){
		numOfChords = numberOfWaypoints;
		this.chordTime = chordTime;
		endingLocation = end;
		dynamicChordNumber = false;
	}
	
	public PathFollowOne(Point2D end){
		endingLocation = end;
		dynamicChordNumber = true;
	}

	@Override
	protected void initialize() {
		double[] startPose = new double[3];
		startPose[0] = Robot.positionTracker.getX();
		startPose[1] = Robot.positionTracker.getY();
		startPose[2] = RobotMap.continuousGyro.getAngle();
		if(dynamicChordNumber){
			numOfChords = Math.sqrt(((startPose[0]-endingLocation.x)*(startPose[0]-endingLocation.x)) + ((startPose[1]-endingLocation.y)*(startPose[1]-endingLocation.y)))/2.0;
			numOfChords = numOfChords <= 1.0 ? 1.0 : numOfChords;
			fakeTimeStep = fakeTime/numOfChords;
			chordTime = fakeTimeStep * 2.0;
		}else{
			numOfChords = numOfChords <= 1.0 ? 1.0 : numOfChords;
			fakeTimeStep = fakeTime/numOfChords;
		}
		double[][] waypoints = {
				{startPose[0],startPose[1]},
				{(startPose[0]+endingLocation.x)/2.0,(startPose[1]+endingLocation.y)/2.0},
				{endingLocation.x,endingLocation.y}
		};
		double[] headings = {startPose[2],0.0};
		FalconPathPlanner planner;
		try {
			planner = new FalconPathPlanner(waypoints,headings);
			planner.calculate(fakeTime, fakeTimeStep, whealSeperationInches);
			pathNodes = planner.smoothPath;
		} catch (Exception e) {
			end();
			e.printStackTrace();
		}
	}

	@Override
	protected void execute() {
		new CommandGroup(){{
			for(int i = 0; i < pathNodes.length; i++){
				Point2D lemLocation = new Point2D(pathNodes[i][0], pathNodes[i][1]);
				segmentType type;
				if(i == 0 && i == pathNodes.length -1){
					type = segmentType.NAT;
				}else if(i == 0){
					type = segmentType.START;
				}if(i == pathNodes.length -1){
					type = segmentType.END;
				}else{
					type = segmentType.INT;
				}
				addSequential(new DriveStraitToFieldPosition(lemLocation,type,tempSpeedNumber,chordTime));
			}
		}}.start();
	}
	
	@Override
	protected boolean isFinished() {
		return true;
	}

	@Override
	protected void end() {
	}

	@Override
	protected void interrupted() {
		end();
	}
}
