package org.usfirst.frc.team948.robot.subsystems;

import org.usfirst.frc.team948.robot.RobotMap;

import edu.wpi.first.wpilibj.command.Subsystem;

/**
 *
 */

public class Shooter extends Subsystem {

    // Put methods for controlling this subsystem
    // here. Call these from Commands.

    public void initDefaultCommand() {
        // Set the default command for a subsystem here.
        //setDefaultCommand(new MySpecialCommand());
    }

	public void shoot(double power) {
		RobotMap.shooterTopMotor.set(power);
		RobotMap.shooterBottomMotor.set(power);
	}

	public void stop() {
		RobotMap.shooterTopMotor.disable();
		RobotMap.shooterBottomMotor.disable();
	}

}

