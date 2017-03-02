package org.usfirst.frc.team948.robot.commands;

import org.usfirst.frc.team948.robot.DS2016;
import org.usfirst.frc.team948.robot.Robot;
import org.usfirst.frc.team948.utilities.MathUtilities;

import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class ManualDrive extends Command {
	public ManualDrive() {
		this.requires(Robot.drive);
	}

	@Override
	protected void initialize() {
	}

	@Override
	protected boolean isFinished() {
		return false;
	}

	@Override
	protected void execute() {
		double leftJoystick = MathUtilities.deadband(DS2016.leftJS.getY(), 0.1);
		double rightJoystick = MathUtilities.deadband(DS2016.rightJS.getY(), 0.1);
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
