package org.usfirst.frc.team948.robot.commands;

import org.usfirst.frc.team948.robot.Robot;
import org.usfirst.frc.team948.robot.RobotMap;
import org.usfirst.frc.team948.robot.visionField;
import org.usfirst.frc.team948.robot.visionProc;
import org.usfirst.frc.team948.robot.visionproccesor;
import org.usfirst.frc.team948.robot.subsystems.Drive;
import org.usfirst.frc.team948.robot.subsystems.Drive.Direction;
import org.usfirst.frc.team948.utilities.PreferenceKeys;

import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.command.CommandGroup;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * Robot drives straight using autonomousHeading. To go forward use positive
 * distance and positive power. To go backwards use positive distance and
 * negative power.
 * 
 * autonomousHeading() not working, so switched back to
 * continuousGyro.getHeading()
 */
public class VisionDriveCommandTwo extends Command {
	private final double DEFAULT_AUTONOMOUS_POWER = 0.5;

	private double power;
	private double distance;

	private double encoderLeftStart;
	private double encoderRightStart;

	private visionproccesor proccesor;
	
	private double ticksToTravel;
	private boolean noneDetected = false;
	private boolean setHeading;
	private double ticksTraveled;
	private double desiredHeading;
	
	private double gamma = 0.0;

	private Direction direction;
	
	public VisionDriveCommandTwo(double DEPRECATEDdistance,double power, visionproccesor proccesor, boolean setHeading) {
		this.direction = direction.FORWARD;
		this.proccesor = proccesor;
		this.power = Math.abs(power);
		this.distance = Math.abs(distance);
		this.setHeading = setHeading;
	}

	public VisionDriveCommandTwo(double power, visionproccesor proccesor, boolean setHeading) {
		this(0.0, power, proccesor, setHeading);
	}

	public VisionDriveCommandTwo(visionproccesor proccesor, boolean setHeading) {
		this(0.0, proccesor, setHeading);
	}
	
	public VisionDriveCommandTwo(visionproccesor proccesor) {
		this(0.0, proccesor, true);
	}

	@Override
	protected void initialize() {
		if(proccesor.dataExists()){
			visionField a = proccesor.getData();
			if (setHeading) {
				double temp = RobotMap.continuousGyro.getAngle();
				gamma = Math.toDegrees(a.gamma);
				Robot.drive.setAutonomousHeading(temp + gamma);
			}
//			distance = Math.sqrt((a.v*a.v)+((a.v/Math.tan(a.gamma))*(a.v/Math.tan(a.gamma))));
			distance = Math.sqrt(((a.v-7.0)*(a.v-7.0))+((a.v*Math.tan(a.gamma))*(a.v*Math.tan(a.gamma))));
			SmartDashboard.putNumber("FOOBAR",distance);
		}else{
			System.out.println("No objects detected");
			noneDetected = true;
		}
	}

	@Override
	protected void execute() {
		if(!noneDetected){
			new CommandGroup(){{
				addSequential(new Turn(gamma));
				addSequential(new DriveStraightDistance(distance, direction, 1.0));
			}}.start();
		}
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