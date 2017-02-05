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
		camera.setExposureManual(-20);
		proccesor = new visionProc();
		SmartDashboard.putNumber("Time", clock.get());
	}
	public void disabledPeriodic() {
		SmartDashboard.putNumber("Time", clock.get());
		if(!proccesor.objects.isEmpty()){
			SmartDashboard.putBoolean("NoDataOut", false);
			ArrayDeque<double[]> data = proccesor.objects.peekFirst();
			for(int i = 0; data.size() > 0;i++){
				double[] temp = data.pollLast();
				SmartDashboard.putNumber("Object" + i + "X", temp[0]);
				SmartDashboard.putNumber("Object" + i + "Y", temp[1]);
				SmartDashboard.putNumber("Object" + i + "Area", temp[2]);
			}
		}
		else{
			SmartDashboard.putBoolean("NoDataOut", true);
		}
	}
}
