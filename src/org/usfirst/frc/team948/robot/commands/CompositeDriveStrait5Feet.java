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

public class CompositeDriveStrait5Feet extends Command {
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
	
	public CompositeDriveStrait5Feet(){
		dynamicChordNumber = true;
	}

	@Override
	protected void initialize() {
		double[] startPose = new double[3];
		startPose[0] = Robot.positionTracker.getX();
		startPose[1] = Robot.positionTracker.getY();
		startPose[2] = RobotMap.continuousGyro.getAngle();
		endingLocation = new Point2D(startPose[0]+ (60.0*Math.sin(Math.toRadians(startPose[2]))),startPose[1] + (60.0*Math.cos(Math.toRadians(startPose[2]))));
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
				{endingLocation.x,endingLocation.y}};
		pathNodes = waypoints;
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
					System.out.println("Making Start segment:" +i);
					type = segmentType.START;
				}if(i == pathNodes.length -1){
					System.out.println("Making End segment:"+ i);
					type = segmentType.END;
				}else{
					System.out.println("Making Int segment:" + i);
					type = segmentType.INT;
				}
				addSequential(new DriveStraitToFieldPosition(lemLocation,type,tempSpeedNumber,-1.0));
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
