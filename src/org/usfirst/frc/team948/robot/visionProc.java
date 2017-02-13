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
import edu.wpi.first.wpilibj.CameraServer;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class visionProc {
	//Real distances are in inches
	public static final boolean bool = false;
	public static final double initialDistance = bool ?  30.0 : 1;
	public static final double initialHeight = bool ? 28.0 : 1;
	public static final double initialWidth = bool ? 10.0 : 1;
	Thread processingThread;
	ConcurrentLinkedDeque<ArrayDeque<double[]>> objects;
	public visionProc(){
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
						// Send the output the error.
						out.notifyError(cvSink.getError());
						// skip the rest of the current iteration
						continue;
					}
					ArrayDeque<double[]> output = new ArrayDeque<double[]>();;
					pipeLine.process(mat);
					ArrayList<MatOfPoint> cameraIn = pipeLine.findContoursOutput();
					int cont = cameraIn.size();
					double[] properties = new double[5];
					int k = 0;
					for(int i = 0; i < cont;i++){
						//double[] properties = new double[3];
						MatOfPoint temp0 = cameraIn.get(i);
	//					properties[2] = temp0.size().area();
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
	//					properties[0] = (temp1.tl().x +temp1.br().x)/2;
						//properties[0] = temp1.width;
	//					properties[1] = (temp1.tl().y +temp1.br().y)/2;
						//properties[1] = temp1.height;
						//Extra processing that is not currently ejected from the tread
						//output.offerFirst(properties);
	//					Imgproc.rectangle(mat, temp1.br(), temp1.tl(),
	//							new Scalar(255, 255, 255), 1);
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
		processingThread.setDaemon(true);
		processingThread.start();
	}
	
//	public double getThetaSingleTape(MatOfPoint in){
//		Rect fitted = Imgproc.boundingRect(in);
//		double A = in.size().area();
//		double W = fitted.width;
//		double H = fitted.height;
//		double int1 = ((H*W)/A)-1;
//		double theta =Math.atan((int1*rectDistance(fitted))/W);
//		return theta;
//	}
//	
//	public double getThetaSingleTape(double[] in){
//		double A = in[2];
//		double W = in[0];
//		double H = in[1];
//		double int1 = ((H*W)/A)-1;
//		double theta =Math.atan((int1*rectDistance(in))/W);
//		return theta;
//	}
	
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
//		return Math.sqrt(initialHeight/H)*initialDistance;
		return (initialHeight*initialDistance)/H;
	}
	
	public double rectDistance(double[] in){
		double H = in[1];
//		return Math.sqrt(initialHeight/H)*initialDistance;
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
//		double y = (fitted.tl().y+fitted.br().y)/2;
		double pixelsToInchs = Math.sqrt((H*W)/(10*Math.cos(theta)));
		double epsilon = (640/2) - x;
		double gamma = Math.atan((pixelsToInchs*epsilon)/rectDistance(fitted));
		return gamma;
	}
	
	public double getHeadingOffeset(double[] in, double theta){
		double H = in[1];
		double W = in[0];
		double x = in[3];
//		double y = in[4];
		double pixelsToInchs = Math.sqrt((H*W)/(10*Math.cos(theta)));
		double epsilon = (640/2) - x;
		double gamma = Math.atan((pixelsToInchs*epsilon)/rectDistance(in));
		return gamma;
	}
}