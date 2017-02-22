package org.usfirst.frc.team948.robot;

import edu.wpi.cscore.CvSink;
import edu.wpi.cscore.CvSource;
import edu.wpi.cscore.UsbCamera;
import edu.wpi.first.wpilibj.CameraServer;
import edu.wpi.first.wpilibj.IterativeRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import java.util.ArrayDeque;

import org.opencv.core.Mat;
import org.usfirst.frc.team948.pipeline.SimpleEX;
import org.usfirst.frc.team948.robot.visionProc;

/**
 * This is a demo program showing the use of OpenCV to do vision processing. The
 * image is acquired from the USB camera, then a rectangle is put on the image and
 * sent to the dashboard. OpenCV has many methods for different types of
 * processing.
 */
public class Robot extends IterativeRobot {
	private static visionProc proccesor;
	private static Timer clock = new Timer();
	public final double tickDistance = 30;
	@Override
	public void robotInit() {
		clock.start();
		UsbCamera camera = CameraServer.getInstance().startAutomaticCapture();
		camera.setExposureManual(-11);
//		camera.setResolution(640, 380);
		proccesor = new visionProc().start();
		SmartDashboard.putNumber("Time", clock.get());
	}
	public void disabledPeriodic() {
		SmartDashboard.putNumber("Time", clock.get());
		if(proccesor.dataExists()){
			SmartDashboard.putBoolean("NoDataOut", false);
			visionField data = proccesor.getData();
			SmartDashboard.putNumber("Theta", (data.theta*180.0)/Math.PI);
			SmartDashboard.putNumber("V", data.v);
			SmartDashboard.putNumber("Gamma", (data.gamma*180.0)/Math.PI);
			SmartDashboard.putNumber("Zeta", data.zeta);
			SmartDashboard.putNumber("Omega", data.omega);
		}
		else{
			SmartDashboard.putBoolean("NoDataOut", true);
		}
	}
}
