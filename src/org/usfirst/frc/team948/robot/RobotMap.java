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
	public static AHRS navx = new AHRS(SPI.Port.kMXP);
	public static ContinuousGyro continuousGyro = new ContinuousGyro(navx);
	public static MeshedEncoders meshedEncoder;
	public RobotMap(){
		preferences = Preferences.getInstance();
		frontLeftMotor =new Victor(frontLeftMotorPort);
		frontRightMotor = new Victor(frontRightMotorPort);
		backLeftMotor = new Victor(backLeftMotorPort);
		backRightMotor = new Victor(backRightMotorPort);
		leftEncoder = new Encoder(leftEncoderPortOne, leftEncoderPortTwo, true);
		rightEncoder = new Encoder(rightEncoderPortOne, rightEncoderPortTwo);
		meshedEncoder = new MeshedEncoders(this);
	}
}
