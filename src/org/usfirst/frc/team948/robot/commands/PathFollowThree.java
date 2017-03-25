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
public class PathFollowThree extends Command {
	private static final double SLOW_DOWN_DISTANCE = 1.0;
	private static final double KDELTA = 180;
	private Point2D[] path;
	private double xTarget;
	private double yTarget;
	private double currentTheta;
	private double thetaToTargetCurrent;
	private double xCurrent;
	private double yCurrent;
	private double distanceToTargetCurrent;
	private int index = 0;
	public PathFollowThree(Point2D[] path) {
		this.path = path;
		requires(Robot.drive);
	}
	
	protected void getFromTracker(){
		xTarget = path[index].x;
		yTarget = path[index].y;
		xCurrent = Robot.positionTracker.getX();
		yCurrent = Robot.positionTracker.getY();
		currentTheta = RobotMap.continuousGyro.getAngle();
		thetaToTargetCurrent = Math.toDegrees(Math.atan2(xTarget - xCurrent, yTarget - yCurrent));
		distanceToTargetCurrent = Math.sqrt((xTarget - xCurrent) * (xTarget - xCurrent) + (yTarget - yCurrent) * (yTarget - yCurrent));
		System.out.println("Possition: (" + xCurrent +"," + yCurrent + ") \n Target: (" + xTarget + "," + yTarget + ")");
	}

	// Called just before this Command runs the first time
	protected void initialize() {
		Robot.drive.driveOnHeadingInit(RobotMap.continuousGyro.getAngle());
	}

	// Called repeatedly when this Command is scheduled to run
	protected void execute() {
		getFromTracker();
		double normalizedHeadingDelta = (thetaToTargetCurrent - currentTheta)/180.0;
		double updatedHeading;
		if (distanceToTargetCurrent > 3.0 && Math.abs(normalizedHeadingDelta) > 3.0/180.0) {
			updatedHeading = RobotMap.continuousGyro.getAngle() - KDELTA * normalizedHeadingDelta;
		} else {
			updatedHeading = Robot.drive.getAutonomousHeading();
		}
		double distanceToSlow = RobotMap.preferences.getDouble("SLOW_DOWN_DISTANCE", SLOW_DOWN_DISTANCE);
		double power;
		power = MathUtil.clamp(distanceToTargetCurrent / distanceToSlow, 0.1, 0.5);
//		if(Math.abs(normalizedHeadingDelta) < 0.2){
//			updatedHeading = RobotMap.continuousGyro.getAngle();
//		}
		Robot.drive.driveOnHeadingLazy(power, updatedHeading);
	}

	// Make this return true when this Command no longer needs to run execute()
	protected boolean isFinished() {
		if(distanceToTargetCurrent < 1.0){
			if(index == path.length - 1){
				return true;
			}else{
				index++;
			}
		}
		return false;
	}

	// Called once after isFinished returns true
	protected void end() {
		Robot.drive.driveOnHeadingEnd(true);
	}

	// Called when another command which requires one or more of the same
	// subsystems is scheduled to run
	protected void interrupted() {
		end();
	}	
}
