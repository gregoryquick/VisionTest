package org.usfirst.frc.team948.robot;

import org.usfirst.frc.team948.robot.commands.CompositeDriveStrait5Feet;
import org.usfirst.frc.team948.robot.commands.DriveStraightDistance;
import org.usfirst.frc.team948.robot.commands.DriveStraitToFieldPosition;
import org.usfirst.frc.team948.robot.commands.ManualDrive;
import org.usfirst.frc.team948.robot.commands.ManualDriveStraight;
import org.usfirst.frc.team948.robot.commands.PathFollowOne;
import org.usfirst.frc.team948.robot.commands.PathFollowTwo;
import org.usfirst.frc.team948.robot.commands.SimpleDriveToPose;
import org.usfirst.frc.team948.robot.commands.VisionDriveCommandOne;
import org.usfirst.frc.team948.robot.commands.VisionDriveCommandTwo;
import org.usfirst.frc.team948.robot.commands.DriveStraitToFieldPosition.segmentType;
import org.usfirst.frc.team948.robot.commands.VisionDriveContOne;
import org.usfirst.frc.team948.robot.commands.VisionDriveContTwo;
import org.usfirst.frc.team948.robot.subsystems.Drive;
import org.usfirst.frc.team948.utilities.Point2D;
import org.usfirst.frc.team948.utilities.PositionTracker;

import edu.wpi.cscore.UsbCamera;
import edu.wpi.first.wpilibj.CameraServer;
import edu.wpi.first.wpilibj.IterativeRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.command.Scheduler;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * This is a demo program showing the use of OpenCV to do vision processing. The
 * image is acquired from the USB camera, then a rectangle is put on the image
 * and sent to the dashboard. OpenCV has many methods for different types of
 * processing.
 */
public class Robot extends IterativeRobot {
	private static final boolean boooool = true;
	public static final int Camera_Width = 160;
	public static final int Camera_Height = 120;
	static VisionProccesor proccesor;
	private static Timer clock = new Timer();
	public final double tickDistance = 30;
	public static OI oi;
	public static Drive drive;
	public static RobotMap robotMap;
	public static PositionTracker positionTracker = new PositionTracker();

	@Override
	public void robotInit() {
		clock.start();
		robotMap = new RobotMap();
		System.out.println("Starting Position Tracker");
		drive = new Drive();
		OI.buttonInit();
		UsbCamera camera = CameraServer.getInstance().startAutomaticCapture();
		camera.setExposureManual(-11);
		camera.setResolution(Camera_Width, Camera_Height);
		proccesor = boooool ? new visionProc().start() : new NewVisionProc().start();
		SmartDashboard.putNumber("Time", clock.get());
		SmartDashboard.putData("ManualDrive", new ManualDrive());
		SmartDashboard.putData("DriveStraight", new ManualDriveStraight());
		SmartDashboard.putData("DriveStraight 5 feet", new DriveStraightDistance(60.0, Drive.Direction.FORWARD));
//		SmartDashboard.putData("Drive With Vision One", new VisionDriveCommandOne(0.3, proccesor, true));
//		SmartDashboard.putData("Drive With Vision Two", new VisionDriveCommandTwo(0.3, proccesor, true));
//		SmartDashboard.putData("Drive With Vision Three", new VisionDriveContOne(0.3, proccesor));
//		SmartDashboard.putData("Drive With Vision Four", new VisionDriveContTwo(0.3, proccesor));
		SmartDashboard.putData("Drive spline to place", new PathFollowTwo(new Point2D(-48.0,120.0),0.0));
		SmartDashboard.putData("Drive Composite 5 feet", new CompositeDriveStrait5Feet());
		SmartDashboard.putData("Pose Change (-120, 240, 0.0)", new SimpleDriveToPose(new Point2D(-120.0,240.0), -1.0));
		SmartDashboard.putData("Drive strait to (-24, 120)",new DriveStraitToFieldPosition(new Point2D(-24.0,120.0)));
		SmartDashboard.putData("Test drive Command",new DriveStraitToFieldPosition(new Point2D(-24.0,120.0),segmentType.START));
	}
	
	public void teleopInit(){
		positionTracker.reset();
		RobotMap.leftEncoder.reset();
		RobotMap.rightEncoder.reset();
		RobotMap.navx.reset();
		positionTracker.start();
	}

	public void teleopPeriodic() {
		periodicAll();
		Scheduler.getInstance().run();
	}
	
	public void disabledInit(){
		positionTracker.stop();
		RobotMap.leftEncoder.reset();
		RobotMap.rightEncoder.reset();
		RobotMap.navx.reset();
	}

	public void disabledPeriodic() {
		periodicAll();
	}
	
	public void periodicAll(){
		SmartDashboard.putNumber("leftEncoder", robotMap.leftEncoder.get());
		SmartDashboard.putNumber("rightEncoder", robotMap.rightEncoder.get());
		SmartDashboard.putNumber("Yaw", robotMap.navx.getAngle());
		SmartDashboard.putNumber("Time", clock.get());
		SmartDashboard.putString("Stringed Tracker: ", positionTracker.toString());
//		SmartDashboard.putNumber("Ultrasonic zero",(robotMap.ultrasoundone.getVoltage() - RobotMap.preferences.getDouble("ultra0add", 0.0255)) / RobotMap.preferences.getDouble("ultra0divide", 0.0242));
//		SmartDashboard.putNumber("Ultrasonic one",(robotMap.ultrasoundtwo.getVoltage() - RobotMap.preferences.getDouble("ultra1add", 0.0255)) / RobotMap.preferences.getDouble("ultra1divide", 0.0242));
		//DO NOT UNCOMMENT THIS CODE
//		if (proccesor.dataExists()) {
//			SmartDashboard.putBoolean("NoDataOut", false);
//			visionField data = proccesor.getData();
//			SmartDashboard.putNumber("Theta", (data.theta * 180.0) / Math.PI);
//			SmartDashboard.putNumber("V", data.v);
//			SmartDashboard.putNumber("Gamma", (data.gamma * 180.0) / Math.PI);
//			SmartDashboard.putNumber("Zeta", data.zeta);
//			SmartDashboard.putNumber("Omega", data.omega);
//		} else {
//			SmartDashboard.putBoolean("NoDataOut", true);
//		}
	}
}
