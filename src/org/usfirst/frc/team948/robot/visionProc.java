package org.usfirst.frc.team948.robot;

import java.util.ArrayDeque;
import java.util.ArrayList;
import java.util.concurrent.ConcurrentLinkedDeque;

import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.usfirst.frc.team948.pipeline.SimpleEX;
import org.usfirst.frc.team948.pipeline.HSimpleEX;
import org.usfirst.frc.team948.pipeline.Pipe;

import edu.wpi.cscore.CvSink;
import edu.wpi.cscore.CvSource;
import edu.wpi.cscore.VideoSink;
import edu.wpi.cscore.VideoSource;
import edu.wpi.first.wpilibj.CameraServer;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class visionProc {
	//Real distances are in inches
	public static final boolean bool = true;
	public static final double initialDistance = bool ?  30.0 : 34.5;
	public static final double initialHeight = bool ? 28.0 : 24.0;
	public static final double initialWidth = bool ? 10.0 : 11.0;
	Thread processingThread;
	ConcurrentLinkedDeque<ArrayDeque<double[]>> objects;
	public visionProc(){}
	
	public visionProc start(){
		objects = new ConcurrentLinkedDeque<ArrayDeque<double[]>>();
		processingThread = new Thread(() -> {
			Pipe pipeLine = bool ? new SimpleEX() : new HSimpleEX();
			CvSink cvSink = CameraServer.getInstance().getVideo();
			CvSource out = CameraServer.getInstance().putVideo("Processed", 640, 480);
			Mat mat = new Mat();
			Timer timer = new Timer();
			timer.start();
			cvSink.grabFrame(mat);
			while (!Thread.interrupted()) {
				SmartDashboard.putNumber("Timer", timer.get());
				if(timer.get() > 0.002){
					if (cvSink.grabFrame(mat) == 0) {
						SmartDashboard.putBoolean("frameError", true);
						out.notifyError(cvSink.getError());
						continue;
					}
					SmartDashboard.putBoolean("frameError", false);
					ArrayDeque<double[]> output = new ArrayDeque<double[]>();;
					pipeLine.process(mat);
					ArrayList<MatOfPoint> cameraIn = pipeLine.findContoursOutput();
					int cont = cameraIn.size();
					double[] properties = new double[5];
					int k = 0;
					for(int i = 0; i < cont;i++){
						MatOfPoint temp0 = cameraIn.get(i);
						Rect temp1 = Imgproc.boundingRect(temp0);
						if(i != 0){
							if(temp1.area() > properties[0]*properties[1]){
								k= i;
								properties[2] = temp0.size().area();
								properties[1] = temp1.height;
								properties[0] = temp1.width;
								properties[3] = (temp1.tl().x + temp1.br().x)/2;
								properties[4] = (temp1.tl().y + temp1.br().y)/2;
							}
						}else{
							properties[2] = temp0.size().area();
							properties[1] = temp1.height;
							properties[0] = temp1.width;
							properties[3] = (temp1.tl().x + temp1.br().x)/2;
							properties[4] = (temp1.tl().y + temp1.br().y)/2;
						}
					}
					if(cont > 0){
						Rect j = Imgproc.boundingRect(cameraIn.get(k));
						Imgproc.rectangle(mat, j.br(), j.tl(), new Scalar(255, 255, 255), 1);
						output.offerFirst(properties);
					}
					objects.addFirst(output);
					timer.reset();
				}
				out.putFrame(mat);
			}
		});
//		processingThread = new Thread(() -> {
//			CvSink cvSink = CameraServer.getInstance().getVideo();
//			CvSource out = CameraServer.getInstance().putVideo("Processed", 640, 480);
//			Mat mat = new Mat();
//			while(!Thread.interrupted()){
//				if(cvSink.grabFrame(mat) == 0){
//					out.notifyError(cvSink.getError());
//					continue;
//				}
//				out.putFrame(mat);
//			}
//		});
		processingThread.setDaemon(true);
		processingThread.start();
		return this;
	}
		
	public double getThetaSingleTape(double[] in){
		SmartDashboard.putBoolean("ThetaTest1", true);
		double W = in[0];
		double H = in[1];
		double distance = rectDistance(in);
		SmartDashboard.putBoolean("ThetaTest2", true);
		double uW = (initialDistance/distance)*initialWidth;
		SmartDashboard.putNumber("ThetaTest3", uW);
		SmartDashboard.putNumber("ThetaTest4", W);
		double theta = Math.acos(W/uW);
		SmartDashboard.putNumber("ThetaTest5", theta);
		return theta;
	}
	
	public double rectDistance(Rect in){
		double H = in.height;
		return (initialHeight*initialDistance)/H;
	}
	
	public double rectDistance(double[] in){
		double H = in[1];
		return (initialHeight*initialDistance)/H;
	}
	
	public double getCenterDistance(MatOfPoint in, double theta){
		Rect fitted = Imgproc.boundingRect(in);
		double closestDistance = rectDistance(fitted);
		return closestDistance + Math.sin(theta);
	}
	
	public double getCenterDistance(double[] in, double theta){
		double closestDistance = rectDistance(in);
		return closestDistance + Math.sin(theta);
	}
	
	public double getHeadingOffeset(MatOfPoint in, double theta){
		Rect fitted = Imgproc.boundingRect(in);
		double H = fitted.height;
		double W = fitted.width;
		double x = (fitted.tl().x+fitted.br().x)/2;
		double epsilon = x - (640/2);
		double gamma = Math.atan((2*epsilon)/(initialDistance*initialWidth));
		return gamma;
	}
	
	public double getHeadingOffeset(double[] in, double theta){
		double x = in[3];
		double epsilon = x - (640/2);
		double gamma = Math.atan((2*epsilon)/(initialDistance*initialWidth));
		return gamma;
	}
}