package org.usfirst.frc.team948.robot;

import java.util.ArrayDeque;
import java.util.ArrayList;

import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Rect;
import org.opencv.imgproc.Imgproc;
import org.usfirst.frc.team948.pipeline.SimpleEX;

import edu.wpi.cscore.CvSink;
import edu.wpi.first.wpilibj.CameraServer;

public class visionProc {
	Thread processingThread;
	ArrayDeque<double[]> objects;
	public visionProc(){
		SimpleEX pipeLine = new SimpleEX();
		objects = new ArrayDeque<double[]>();
		processingThread = new Thread(() -> {
			CvSink cvSink = CameraServer.getInstance().getVideo();
			Mat mat = new Mat();
			while (!Thread.interrupted()) {
				ArrayDeque<double[]> output = new ArrayDeque<double[]>();
				cvSink.grabFrame(mat);
				pipeLine.process(mat);
				ArrayList<MatOfPoint> cameraIn = pipeLine.findContoursOutput();
				int cont = cameraIn.size();
				for(int i = 0; i < cont;i++){
					double[] properties = new double[2];
					MatOfPoint temp0 = cameraIn.get(i);
					properties[2] = temp0.size().area();
					Rect temp1 = Imgproc.boundingRect(temp0);
					properties[0] = (temp1.tl().x +temp1.br().x)/2;
					properties[1] = (temp1.tl().y +temp1.br().y)/2;
					output.offerFirst(properties);
				}
				setObjects(output);
			}
		});
		processingThread.setDaemon(true);
		processingThread.start();
	}
	
	public void setObjects(ArrayDeque<double[]> input){
		synchronized(objects){
			objects = new ArrayDeque<double[]>();
			while(input.size() > 0){
				double[] content = input.pollLast();
				objects.offerFirst(content);
			}
		}
	}
	
	public ArrayDeque<double[]> getObjects() throws InterruptedException{
		pause(2);
		synchronized(objects){
			return objects;
		}
	}
	
	public double[] weightedObjectAverage() throws InterruptedException{ 
		ArrayDeque<double[]> temp;
		pause(2);
		synchronized(objects){
			temp = objects.clone();
		}
		if(temp.size() != 0){
			double[] a = {0,0,0};
			double number = (double) temp.size();
			while(temp.size() > 0){
				double[] f = temp.pollFirst();
				a[0] += f[0]*f[2];
				a[1] +=	f[1]*f[2];
				a[2] += f[2];
			}
			a[0] /= number*a[2];
			a[1] /= number*a[2];
			a[2] /= number;
			return a;
		}
		else{
			return null;
		}
	}
		
	public double[] objectAverage() throws InterruptedException{
		ArrayDeque<double[]> temp;
		pause(2);
		synchronized(objects){
			temp = objects.clone();
		}
		if(temp.size() != 0){
			double[] a = {0,0,0};
			double number = (double) temp.size();
			while(temp.size() > 0){
				double[] f = temp.pollFirst();
				a[0] += f[0];
				a[1] +=	f[1];
				a[2] += f[2];
			}
			a[0] /= number;
			a[1] /= number;
			a[2] /= number;
			return a;
		}
		else{
			return null;
		}
	}
	
	public void pause() throws InterruptedException{
		processingThread.wait();
	}
	
	public void pause(long time) throws InterruptedException{
		processingThread.sleep(time);
	}
	
	public void unpause(){
		notifyAll();
	}
}