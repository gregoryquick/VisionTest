package org.usfirst.frc.team948.robot;

import org.usfirst.frc.team948.utilities.ContinuousGyro;
import org.usfirst.frc.team948.utilities.MeshedEncoders;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Preferences;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.Victor;

public class RobotMap {
	private static final int frontLeftMotorPort = 0;
	private static final int frontRightMotorPort = 2;
	private static final int backLeftMotorPort = 1;
	private static final int backRightMotorPort = 3;
	private static final int leftEncoderPortOne = 0;
	private static final int leftEncoderPortTwo = 1;
	private static final int rightEncoderPortOne = 2;
	private static final int rightEncoderPortTwo = 3;
	public static Victor frontLeftMotor ;
	public static Victor frontRightMotor;
	public static Victor backLeftMotor;
	public static Victor backRightMotor;
	public static Encoder leftEncoder;
	public static Encoder rightEncoder;
	public static Preferences preferences;
	public static ContinuousGyro navX;
	public static MeshedEncoders meshedEncoder;
	public RobotMap(){
		preferences = Preferences.getInstance();
		frontLeftMotor =new Victor(preferences.getInt("frontLeftMotorPort", frontLeftMotorPort));
		frontRightMotor = new Victor(preferences.getInt("frontRightMotorPort", frontRightMotorPort));
		backLeftMotor = new Victor(preferences.getInt("backLeftMotorPort", backLeftMotorPort));
		backRightMotor = new Victor(preferences.getInt("backRightMotorPort", backRightMotorPort));
		frontLeftMotor.setInverted(true);
		backLeftMotor.setInverted(true);
		frontRightMotor.setInverted(false);
		backLeftMotor.setInverted(false);
		navX = new ContinuousGyro(SPI.Port.kMXP);
		leftEncoder = new Encoder(preferences.getInt("leftEncoderPortOne", leftEncoderPortOne),preferences.getInt("leftEncoderPortTwo", leftEncoderPortTwo));
		rightEncoder = new Encoder(preferences.getInt("rightEncoderPortOne", rightEncoderPortOne),preferences.getInt("rightEncoderPortTwo", rightEncoderPortTwo));
		leftEncoder.setReverseDirection(preferences.getBoolean("leftEncoderInverted",true));
		rightEncoder.setReverseDirection(preferences.getBoolean("rightEncoderInverted",false));
		meshedEncoder = new MeshedEncoders(this);
	}
}
