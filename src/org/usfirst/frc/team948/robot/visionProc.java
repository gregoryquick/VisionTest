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

import edu.wpi.cscore.CvSink;
import edu.wpi.cscore.CvSource;
import edu.wpi.first.wpilibj.CameraServer;

public class visionProc {
	//Real distances are in inches
	
	//<To set later>
	public static final double initialDistance = 38.0;
	public static final double initialHeight = 23.0;
	//</To set later>
	Thread processingThread;
	ConcurrentLinkedDeque<ArrayDeque<double[]>> objects;
	public visionProc(){
		objects = new ConcurrentLinkedDeque<ArrayDeque<double[]>>();
		processingThread = new Thread(() -> {
			SimpleEX pipeLine = new SimpleEX();
			CvSink cvSink = CameraServer.getInstance().getVideo();
			CvSource out = CameraServer.getInstance().putVideo("Processed", 640, 480);
			Mat mat = new Mat();
			while (!Thread.interrupted()) {
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
				double[] properties = new double[3];
				int k = 0;
				for(int i = 0; i < cont;i++){
					//double[] properties = new double[3];
					MatOfPoint temp0 = cameraIn.get(i);
//					properties[2] = temp0.size().area();
					Rect temp1 = Imgproc.boundingRect(temp0);
					if(i != 0){
						if(temp0.size().area() > properties[2]){
							k= i;
							properties[2] = temp0.size().area();
							properties[1] = temp1.height;
							properties[0] = temp1.width;
						}
					}else{
						k = i;
						properties[2] = temp0.size().area();
						properties[1] = temp1.height;
						properties[0] = temp1.width;
					}
//					properties[0] = (temp1.tl().x +temp1.br().x)/2;
					//properties[0] = temp1.width;
//					properties[1] = (temp1.tl().y +temp1.br().y)/2;
					//properties[1] = temp1.height;
					//Extra processing that is not currently ejected from the tread
					if(Thread.interrupted()){
						double theta = getThetaSingleTape(temp0);
						
					}
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
				out.putFrame(mat);
			}
		});
		processingThread.setDaemon(true);
		processingThread.start();
	}
	
	public double getThetaSingleTape(MatOfPoint in){
		Rect fitted = Imgproc.boundingRect(in);
		double A = in.size().area();
		double W = fitted.width;
		double H = fitted.height;
		double int1 = ((H*W)/A)-1;
		double theta =Math.atan((int1*rectDistance(fitted))/W);
		return theta;
	}
	
	public double rectDistance(Rect in){
		double H = in.height;
//		return Math.sqrt(initialHeight/H)*initialDistance;
		return (initialHeight*initialDistance)/H;
	}
	
	public double getCenterDistance(MatOfPoint in, double theta){
		Rect fitted = Imgproc.boundingRect(in);
		double closestDistance = rectDistance(fitted);
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
}