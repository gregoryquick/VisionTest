package org.usfirst.frc.team948.robot;

import edu.wpi.cscore.CvSink;
import edu.wpi.cscore.CvSource;
import edu.wpi.cscore.UsbCamera;
import edu.wpi.first.wpilibj.CameraServer;
import edu.wpi.first.wpilibj.IterativeRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import java.util.ArrayList;

import org.opencv.core.*;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Point;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.opencv.imgproc.Moments;
import org.usfirst.frc.team948.pipeline.SimpleEX;
import org.usfirst.frc.team948.pipeline.WebCamPipeVid;

/**
 * This is a demo program showing the use of OpenCV to do vision processing. The
 * image is acquired from the USB camera, then a rectangle is put on the image and
 * sent to the dashboard. OpenCV has many methods for different types of
 * processing.
 */
public class Robot extends IterativeRobot {
//	Thread visionThread;
//	CvSource outputStream;
//	SimpleEX pipeLine = new SimpleEX();
	static Timer clock = new Timer();
	public final double tickDistance = 30;
	@Override
	public void robotInit() {
//		visionThread = new Thread(() -> {
//			UsbCamera camera = CameraServer.getInstance().startAutomaticCapture();
//			camera.setResolution(640, 480);
//			CvSink cvSink = CameraServer.getInstance().getVideo();
//			outputStream = CameraServer.getInstance().putVideo("Rectangle", 640, 480);
//			Mat mat = new Mat();
//			while (!Thread.interrupted()) {
//				if (cvSink.grabFrame(mat) == 0) {
//					outputStream.notifyError(cvSink.getError());
//					continue;
//				}
//				pipeLine.process(mat);
//				outputStream.putFrame(pipeLine.hsvThresholdOutput());
//			}
//		});
//		visionThread.setDaemon(true);
//		visionThread.start();
		clock.start();
		}
	
	public void loop(double prevTime){
		//This code is realy mainly just how to find the center, not real code that should be used in a robot
		while(prevTime+tickDistance <= clock.get()){continue;};
//		MatOfPoint In = LargestCont(pipeLine.findContoursOutput());
//		Rect rectange = Imgproc.boundingRect(In);
//		double x = (rectange.tl().x +rectange.br().x)/2;
//		double y = (rectange.tl().y +rectange.br().y)/2;
//		double area = In.size().area();
//		SmartDashboard.putNumber("ObjectX", x);
//		SmartDashboard.putNumber("ObjectY", y);
//		SmartDashboard.putNumber("ObjectArea", area);
	}
	
//	public MatOfPoint LargestCont(ArrayList<MatOfPoint> input){
//		int AM = 0;
//		MatOfPoint output = null;
//		for(MatOfPoint a : input){
//			if(a.size().area() < AM)
//				continue;
//			output = a;
//		}
//		return output;
//	}
}
