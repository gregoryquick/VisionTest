package org.usfirst.frc.team948.robot;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.Preferences;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.Victor;

public class RobotMap {
	private static final int frontLeftMotorPort = 0;
	private static final int frontRightMotorPort = 2;
	private static final int backLeftMotorPort = 3;
	private static final int backRightMotorPort = 1;
	public Victor frontLeftMotor;
	public Victor frontRightMotor;
	public Victor backLeftMotor;
	public Victor backRightMotor;
	public Preferences preferences;
	public AHRS navX;
	public RobotMap(){
		preferences = Preferences.getInstance();
		frontLeftMotor =new Victor(preferences.getInt("frontLeftMotorPort", frontLeftMotorPort));
		frontRightMotor = new Victor(preferences.getInt("frontRightMotorPort", frontRightMotorPort));
		backLeftMotor = new Victor(preferences.getInt("backLeftMotorPort", backLeftMotorPort));
		backRightMotor = new Victor(preferences.getInt("backRightMotorPort", backRightMotorPort));
		navX = new AHRS(SPI.Port.kMXP);
	}
}
