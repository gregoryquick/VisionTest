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
		if(!proccesor.objects.isEmpty()){
			ArrayDeque<double[]> data = proccesor.objects.peekFirst();
			if(data.size() > 0){
				SmartDashboard.putBoolean("NoDataOut", false);
			}else{
				SmartDashboard.putBoolean("NoDataOut", true);
			}
			SmartDashboard.putNumber("DataZise", data.size());
			for(int i = 0; data.size() > 0;i++){
				double[] temp = data.pollLast();
				SmartDashboard.putNumber("Object" + i + "Width", temp[0]);
				SmartDashboard.putNumber("Object" + i + "Height", temp[1]);
				SmartDashboard.putNumber("Object" + i + "X", temp[3]);
				SmartDashboard.putNumber("Object" + i + "Y", temp[4]);
				SmartDashboard.putNumber("Object" + i + "Area", temp[2]);
				SmartDashboard.putNumber("Object" + i + "Size", temp[0]*temp[1]);
				SmartDashboard.putNumber("Object" + i + "Theta", (proccesor.getThetaSingleTape(temp)*180)/Math.PI);
				SmartDashboard.putNumber("Object" + i + "Distance", proccesor.getCenterDistance(temp, proccesor.getThetaSingleTape(temp)));
				SmartDashboard.putNumber("Object" + i + "Gamma", (proccesor.getHeadingOffeset(temp, proccesor.getThetaSingleTape(temp))*180)/Math.PI);
			}
		}
		else{
			SmartDashboard.putBoolean("NoDataOut", true);
		}
	}
}
