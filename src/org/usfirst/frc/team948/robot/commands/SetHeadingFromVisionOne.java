package org.usfirst.frc.team948.robot.commands;

import org.usfirst.frc.team948.robot.OI;
import org.usfirst.frc.team948.robot.Robot;
import org.usfirst.frc.team948.robot.RobotMap;
import org.usfirst.frc.team948.robot.visionField;
import org.usfirst.frc.team948.robot.visionProc;
import org.usfirst.frc.team948.utilities.MathUtil;

public class SetHeadingFromVisionOne extends CommandBase {

	visionProc proccesor;
	
	public SetHeadingFromVisionOne(visionProc proccesor) {
		this.proccesor = proccesor;
	}

	@Override
	protected void initialize() {
		if(proccesor.dataExists()){
			visionField temp = proccesor.getData();
			drive.setAutonomousHeading(RobotMap.continuousGyro.getAngle() + temp.gamma);
		}else{
			System.out.println("No objects detected");
		}
	}

	@Override
	protected boolean isFinished() {
		return false;
	}

	@Override
	protected void execute() {
		double leftJoystick = MathUtil.deadband(-OI.leftJoystick.getY(), 0.1);
		double rightJoystick = MathUtil.deadband(-OI.rightJoystick.getY(), 0.1);
		Robot.drive.tankDrive(leftJoystick, rightJoystick);
	}

	@Override
	protected void end() {
		Robot.drive.stop();
	}

	@Override
	protected void interrupted() {
		end();
	}

}
