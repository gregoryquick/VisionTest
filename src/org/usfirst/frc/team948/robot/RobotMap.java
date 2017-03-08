package org.usfirst.frc.team948.robot;

import org.usfirst.frc.team948.utilities.ContinuousGyro;
import org.usfirst.frc.team948.utilities.MeshedEncoders;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Preferences;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.Solenoid;
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
	private static final int shooterTopMotorPort = 4;
	private static final int shooterBottomMotorPort = 5;
	private static final int shooterTopEncoderPortOne= 4;
	private static final int shooterTopEncoderPortTwo= 5;
	
	public static Solenoid cameraLight = new Solenoid(5);

	public static Victor frontLeftMotor ;
	public static Victor frontRightMotor;
	public static Victor backLeftMotor;
	public static Victor backRightMotor;
	public static Victor shooterTopMotor;
	public static Victor shooterBottomMotor;
	public static Encoder leftEncoder;
	public static Encoder rightEncoder;
	public static Encoder shooterEncoder;
	public static Preferences preferences;
	public static AHRS navx = new AHRS(SPI.Port.kMXP);
	public static ContinuousGyro continuousGyro = new ContinuousGyro(navx);
	public static MeshedEncoders meshedEncoder;
	public static AnalogInput ultrasoundone;
	public static AnalogInput ultrasoundtwo;
	public RobotMap(){
		preferences = Preferences.getInstance();
		frontLeftMotor =new Victor(frontLeftMotorPort);
		frontRightMotor = new Victor(frontRightMotorPort);
		backLeftMotor = new Victor(backLeftMotorPort);
		backRightMotor = new Victor(backRightMotorPort);
		shooterTopMotor = new Victor(shooterTopMotorPort);
		shooterBottomMotor = new Victor(shooterBottomMotorPort);
		leftEncoder = new Encoder(leftEncoderPortOne, leftEncoderPortTwo, true);
		rightEncoder = new Encoder(rightEncoderPortOne, rightEncoderPortTwo);
		shooterEncoder = new Encoder (shooterTopEncoderPortOne, shooterTopEncoderPortTwo);
		meshedEncoder = new MeshedEncoders(this);
		ultrasoundone = new AnalogInput(0);
		ultrasoundtwo = new AnalogInput(1);
	}
}
