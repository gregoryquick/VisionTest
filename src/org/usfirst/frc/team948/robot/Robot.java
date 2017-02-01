package org.usfirst.frc.team948.robot;

import edu.wpi.cscore.CvSink;
import edu.wpi.cscore.CvSource;
import edu.wpi.cscore.UsbCamera;
import edu.wpi.first.wpilibj.CameraServer;
import edu.wpi.first.wpilibj.IterativeRobot;

import java.util.ArrayList;

import org.opencv.core.*;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Point;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.opencv.imgproc.Moments;
import org.usfirst.frc.team948.robot.WebCamPipeVid;

/**
 * This is a demo program showing the use of OpenCV to do vision processing. The
 * image is acquired from the USB camera, then a rectangle is put on the image and
 * sent to the dashboard. OpenCV has many methods for different types of
 * processing.
 */
public class Robot extends IterativeRobot {
	Thread visionThread;

	@Override
	public void robotInit() {
		visionThread = new Thread(() -> {
			PipeTwo pipeTube = new PipeTwo();
			UsbCamera camera = CameraServer.getInstance().startAutomaticCapture();
			camera.setResolution(640, 480);
			CvSink cvSink = CameraServer.getInstance().getVideo();
			CvSource outputStream = CameraServer.getInstance().putVideo("Rectangle", 640, 480);
			Mat mat = new Mat();
			while (!Thread.interrupted()) {
				if (cvSink.grabFrame(mat) == 0) {
					outputStream.notifyError(cvSink.getError());
					continue;
				}
				pipeTube.process(mat);
				outputStream.putFrame(pipeTube.desaturateOutput());
			}
		});
		visionThread.setDaemon(true);
		visionThread.start();
	}
	
	public MatOfPoint LargestCont(ArrayList<MatOfPoint> input){
		int AM = 0;
		MatOfPoint output = null;
		for(MatOfPoint a : input){
			if(a.size().area() < AM)
				continue;
			output = a;
		}
		Moments outputMom = null;
		return output;
	}
}
