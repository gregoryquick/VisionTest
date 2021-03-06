package org.usfirst.frc.team948.robot.commands;
import org.usfirst.frc.team948.robot.OI;
import org.usfirst.frc.team948.robot.Robot;
import org.usfirst.frc.team948.robot.RobotMap;
import org.usfirst.frc.team948.robot.VisionProccesor;
import org.usfirst.frc.team948.robot.visionField;
import org.usfirst.frc.team948.robot.visionProc;
import org.usfirst.frc.team948.robot.subsystems.Drive.Direction;

import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.command.CommandGroup;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class VisionDriveContOne extends Command {
	private double desiredHeading;
	private double power;
	private final double slowingDistance = 10.0;
	private final double stopDistance;
	private VisionProccesor proccesor;
	private visionField field = null;
	private final boolean bool;
	public VisionDriveContOne(double power, VisionProccesor proccesor) {
		this.power = power;
		bool = false;
		this.stopDistance = 15.0;
		this.proccesor = proccesor;
		
		requires(Robot.drive);
	}
	
//	public VisionDriveContOne(double power, visionProc proccesor, double stopDistance) {
//		this.power = power;
//		this.stopDistance = stopDistance;
//		this.proccesor = proccesor;
//		requires(Robot.drive);
//	}

	@Override 
	protected void initialize() {
		if(proccesor.dataExists()){
			field = proccesor.getData();
			desiredHeading = RobotMap.continuousGyro.getAngle();
//			double offsetAngleCorrection = Math.toDegrees(field.gamma);
//			offsetAngleCorrection *= Math.abs(Math.toDegrees(field.gamma)) <= 1 ? 2.0 : Math.max(7.0/(1.0+(1.0/Math.exp(Math.abs(Math.toDegrees(field.gamma))-10.0))),0.15);
			double offsetAngleCorrection = Math.copySign((15.0*(field.v/stopDistance))/(1.0+(1.0/Math.exp(Math.abs(Math.toDegrees(field.gamma))-10.0))),field.gamma);
			desiredHeading += offsetAngleCorrection;
			desiredHeading += 1.0;
			Robot.drive.driveOnHeadingInit(desiredHeading);
		}
	} 
	
	@Override 
	protected void execute(){
		if(proccesor.dataExists()){
			field = proccesor.getData();
			desiredHeading = RobotMap.continuousGyro.getAngle() + Math.toDegrees(field.gamma);
//			double distanceCorrectionAngle = field.zeta*field.v*Math.tan(field.gamma);
//			desiredHeading += distanceCorrectionAngle;
			double currentPower = power * Math.min(1.0,
					Math.max(Math.abs(((field.v) - stopDistance)/(stopDistance + slowingDistance)),0.4));
			SmartDashboard.putNumber("Field V", (field.v));
			SmartDashboard.putNumber("Percent", Math.max(Math.abs(((field.v) - stopDistance)/(stopDistance + slowingDistance)),0.3));
			currentPower = Math.max(currentPower, .3);
			Robot.drive.driveOnHeading(currentPower, desiredHeading);
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
