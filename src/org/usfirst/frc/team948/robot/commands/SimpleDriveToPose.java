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

public class SimpleDriveToPose extends Command {
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
	private double chordTime = -1.0;
	private boolean setChordTime = true;
	public  SimpleDriveToPose(Point2D end,double numberOfWaypoints,double chordTime){
		numOfChords = numberOfWaypoints;
		this.chordTime = chordTime;
		endingLocation = end;
		setChordTime = false;
		dynamicChordNumber = false;
	}
	
	public SimpleDriveToPose(Point2D end, double chordTime){
		endingLocation = end;
		this.chordTime = chordTime;
		setChordTime = false;
		dynamicChordNumber = true;
	}
	
	public SimpleDriveToPose(Point2D end){
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
			if(setChordTime){
				chordTime = fakeTimeStep * 2.0;
			}
		}else{
			numOfChords = numOfChords <= 1.0 ? 1.0 : numOfChords;
			fakeTimeStep = fakeTime/numOfChords;
		}
		double[][] waypoints = {
				{startPose[0],startPose[1]},
				{endingLocation.x,endingLocation.y}
		};
		double[] headings = {startPose[2],0.0};
		double[][] corrected = new double[waypoints.length+2][waypoints[0].length];
		int l = waypoints.length-1;
		double displacement = Math.sqrt(((waypoints[0][0]-waypoints[l][0])*(waypoints[0][0]-waypoints[l][0]))+((waypoints[0][1]-waypoints[l][1])*(waypoints[0][1]-waypoints[l][1])));
		double lambda = Math.tanh(displacement)/10.0;
		System.out.println("Lambda Value:"+ lambda);
		double fudge0 = RobotMap.preferences.getDouble("BeginingPathScaleFudgeFactor", 1.0);
		double fudge1 = RobotMap.preferences.getDouble("EndingPathScaleFudgeFactor", 1.0);
		double[] lemN = new double[]{waypoints[0][0]+ (lambda*fudge0*displacement*Math.sin(Math.toRadians(headings[0]))),waypoints[0][1]+(lambda*fudge0*displacement*Math.cos(Math.toRadians(headings[0])))};
		double[] lemO = new double[]{waypoints[l][0]-(lambda*fudge1*displacement*Math.sin(Math.toRadians(headings[1]))),waypoints[l][1]-(lambda*fudge1*displacement*Math.cos(Math.toRadians(headings[1])))};
		for(int i = 0; i < l+1;i++){
			if(i == 0){
				corrected[0] = waypoints[0];
				corrected[1] = lemN;
			}else if(i == l){
				corrected[i + 1] = lemO;
				corrected[i + 2] = waypoints[i];
			}else{
				corrected[i+1] = waypoints[i];
			}
		}
		pathNodes = corrected;
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
