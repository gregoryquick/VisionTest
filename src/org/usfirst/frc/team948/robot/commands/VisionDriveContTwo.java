package org.usfirst.frc.team948.robot.commands;
import org.usfirst.frc.team948.robot.OI;
import org.usfirst.frc.team948.robot.Robot;
import org.usfirst.frc.team948.robot.RobotMap;
import org.usfirst.frc.team948.robot.VisionProccesor;
import org.usfirst.frc.team948.robot.visionField;
import org.usfirst.frc.team948.robot.visionProc;
import org.usfirst.frc.team948.robot.subsystems.Drive.Direction;
import org.usfirst.frc.team948.utilities.MathUtil;

import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.command.CommandGroup;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class VisionDriveContTwo extends Command {
	private double desiredHeadingOffset;
	private double lastHeadingOffset;
	private double power;
	private final double slowingDistance = 10.0;
	private final double stopDistance;
	private VisionProccesor proccesor;
	private visionField field = null;
	private final boolean bool;
	public VisionDriveContTwo(double power, VisionProccesor proccesor) {
		this.power = power;
		bool = false;
		this.stopDistance = 20.0;
		this.proccesor = proccesor;
		
		requires(Robot.drive);
	}

	@Override 
	protected void initialize() {
		if(proccesor.dataExists()){
			field = proccesor.getData();
			desiredHeadingOffset = 0.0;
			lastHeadingOffset = desiredHeadingOffset;
			Robot.drive.driveOnHeadingInit(desiredHeadingOffset);
		}
	} 
	
	@Override 
	protected void execute(){
		if(proccesor.dataExists()){
			field = proccesor.getData();
			double offsetAngleCorrection = Math.copySign((15.0*(Math.sqrt(field.v)/stopDistance))/(1.0+(1.0/Math.exp(Math.abs(Math.toDegrees(field.gamma))-10.0))),field.gamma);
			double slantCorrectionAngle = Math.copySign((10.0*(stopDistance/field.v))/(1.0+(1.0/Math.exp(Math.abs(Math.toDegrees(field.theta))-10.0))),field.theta);
			desiredHeadingOffset = 0.0;
			desiredHeadingOffset += offsetAngleCorrection;
			desiredHeadingOffset += slantCorrectionAngle;
			desiredHeadingOffset += 1.0;
			double currentPower = power * Math.min(1.0,
					Math.max(Math.abs(((field.v) - stopDistance)/(stopDistance + slowingDistance)),0.4));
			SmartDashboard.putNumber("Field V", (field.v));
			SmartDashboard.putNumber("Percent", Math.max(Math.abs(((field.v) - stopDistance)/(stopDistance + slowingDistance)),0.3));
			currentPower = Math.max(currentPower, .3);
			Robot.drive.driveOnHeading(currentPower, RobotMap.continuousGyro.getAngle() + desiredHeadingOffset);
			lastHeadingOffset = desiredHeadingOffset;
		}else{
//			Robot.drive.stop();
		}
	}
	
	@Override
	protected boolean isFinished() {
		if(field.equals(null))
			return true;
		else
			return (field.v <= (stopDistance  + 2.0));
	}
	
	@Override 
	protected void end() {
		if(!field.equals(null)){
			Robot.drive.driveOnHeadingEnd(true);
			new CommandGroup(){{
					addSequential(new DriveStraightDistance(stopDistance - (5.0*(stopDistance/slowingDistance)), Direction.FORWARD));
				}}.start();
		}
	}
	
	@Override 
	protected void interrupted() {
		Robot.drive.driveOnHeadingEnd(true);
	}
}
