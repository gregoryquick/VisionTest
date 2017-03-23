package org.usfirst.frc.team948.robot.commands;

import org.usfirst.frc.team948.robot.Robot;
import org.usfirst.frc.team948.robot.RobotMap;
import org.usfirst.frc.team948.robot.VisionProccesor;
import org.usfirst.frc.team948.robot.visionField;
import org.usfirst.frc.team948.robot.VisionProccesor;
import org.usfirst.frc.team948.robot.subsystems.Drive.Direction;

import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.command.CommandGroup;

/**
 * Robot drives straight using autonomousHeading. To go forward use positive
 * distance and positive power. To go backwards use positive distance and
 * negative power.
 * 
 * autonomousHeading() not working, so switched back to
 * continuousGyro.getHeading()
 */
public class VisionDriveCommandOne extends Command {
	private final double DEFAULT_AUTONOMOUS_POWER = 0.5;

	private double power;
	private double distance;

	private double encoderLeftStart;
	private double encoderRightStart;

	private VisionProccesor proccesor;
	
	private double ticksToTravel;
	private boolean noneDetected = false;
	private boolean setHeading;
	private double ticksTraveled;
	private double desiredHeading;

	private Direction direction;
	
	public VisionDriveCommandOne(double distance,double power, VisionProccesor proccesor, boolean setHeading) {
		this.direction = direction.FORWARD;
		this.proccesor = proccesor;
		this.power = Math.abs(power);
		this.distance = Math.abs(distance);
		this.setHeading = setHeading;
	}

	public VisionDriveCommandOne(double power, VisionProccesor proccesor, boolean setHeading) {
		this(0.0, power, proccesor, setHeading);
	}

	public VisionDriveCommandOne(VisionProccesor proccesor, boolean setHeading) {
		this(0.0, proccesor, setHeading);
	}
	
	public VisionDriveCommandOne(VisionProccesor proccesor) {
		this(0.0, proccesor, true);
	}

	@Override
	protected void initialize() {
		if(proccesor.dataExists()){
			visionField a = proccesor.getData();
			if (setHeading) {
				double temp = RobotMap.continuousGyro.getAngle();
				Robot.drive.setAutonomousHeading(temp + a.gamma);
			}
			if(distance == 0.0){
//				distance = Math.sqrt((a.v*a.v)+((a.v/Math.tan(a.gamma))*(a.v/Math.tan(a.gamma))));
				distance = Math.sqrt((a.v-5.0)*(a.v-5.0)+((a.v/Math.tan(a.gamma))*(a.v/Math.tan(a.gamma))));
			}
		}else{
			System.out.println("No objects detected");
			noneDetected = true;
		}
	}

	@Override
	protected void execute() {
		if(!noneDetected){
			new CommandGroup(){{
				addSequential(new DriveStraightDistance(distance, direction, power));
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