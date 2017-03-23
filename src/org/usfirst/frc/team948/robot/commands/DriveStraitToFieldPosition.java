package org.usfirst.frc.team948.robot.commands;

import org.usfirst.frc.team948.robot.Robot;
import org.usfirst.frc.team948.robot.RobotMap;
import org.usfirst.frc.team948.utilities.MathUtil;
import org.usfirst.frc.team948.utilities.Point2D;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 *
 */
public class DriveStraitToFieldPosition extends Command {
	private static final double SLOW_DOWN_DISTANCE = 28;
	private static final double KDELTA = 180;
	private final double xTarget;
	private final double yTarget;
	private double currentTheta;
	private double thetaToTargetCurrent;
	private double xCurrent;
	private double yCurrent;
	private double speed;
	private double distanceToTargetCurrent;
	private segmentType type;
	private Timer timekeeper;
	private double timeToFinish = -1.0;

	enum segmentType{
		START, END, INT, NAT;
	}
	public DriveStraitToFieldPosition(Point2D end) {
		xTarget = end.x;
		yTarget = end.y;
		type = segmentType.NAT;
		requires(Robot.drive);
	}
	
	public DriveStraitToFieldPosition(Point2D end,segmentType segment, double power, double time) {
		xTarget = end.x;
		yTarget = end.y;
		speed = power;
		type = segment;
		timeToFinish = time;
		requires(Robot.drive);
	}
	
	protected void getFromTracker(){
		xCurrent = Robot.positionTracker.getX();
		yCurrent = Robot.positionTracker.getY();
		currentTheta = RobotMap.continuousGyro.getAngle();
		thetaToTargetCurrent = Math.toDegrees(Math.atan2(xTarget - xCurrent, yTarget - yCurrent));
		distanceToTargetCurrent = Math.sqrt((xTarget - xCurrent) * (xTarget - xCurrent) + (yTarget - yCurrent) * (yTarget - yCurrent));
		System.out.println("Possition: (" + xCurrent +"," + yCurrent + ") \n Target: (" + xTarget + "," + yTarget + ")");
	}

	// Called just before this Command runs the first time
	protected void initialize() {
		if(type.equals(segmentType.NAT) || type.equals(segmentType.START));
			Robot.drive.driveOnHeadingInit(RobotMap.continuousGyro.getAngle());
		timekeeper = new Timer();
		timekeeper.start();
	}

	// Called repeatedly when this Command is scheduled to run
	protected void execute() {
		getFromTracker();
		double normalizedHeadingDelta = (thetaToTargetCurrent - currentTheta)/180;
		double updatedHeading;
		if (distanceToTargetCurrent > 18 || Math.abs(normalizedHeadingDelta) > 0.1) {
			updatedHeading = RobotMap.continuousGyro.getAngle() + KDELTA * normalizedHeadingDelta;
		} else {
			updatedHeading = Robot.drive.getAutonomousHeading();
		}
		double distanceToSlow = RobotMap.preferences.getDouble("SLOW_DOWN_DISTANCE", SLOW_DOWN_DISTANCE);
		double power;
		if(type.equals(segmentType.NAT)){
			 power = MathUtil.clamp(distanceToTargetCurrent / distanceToSlow, 0.15, 1);
		}else{
			power = MathUtil.clamp(speed, 0.15, 1);
		}
		power = 0.5;
		Robot.drive.driveOnHeading(power, updatedHeading);
	}

	// Make this return true when this Command no longer needs to run execute()
	protected boolean isFinished() {
		// TODO Figure out correct distance
		System.out.println("" + distanceToTargetCurrent);
		if(timeToFinish <= 0.0){
			return timekeeper.get() > timeToFinish;
		}else{
			return distanceToTargetCurrent < 2.0;
		}
//		return false;
	}

	// Called once after isFinished returns true
	protected void end() {
		if(type.equals(segmentType.NAT) || type.equals(segmentType.END))
			Robot.drive.driveOnHeadingEnd(true);
		timekeeper.stop();
	}

	// Called when another command which requires one or more of the same
	// subsystems is scheduled to run
	protected void interrupted() {
		System.out.println("Field position drive was interrupted");
		end();
	}	
}
