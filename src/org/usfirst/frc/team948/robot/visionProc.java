package org.usfirst.frc.team948.robot;

import java.util.ArrayDeque;
import java.util.ArrayList;
import java.util.concurrent.ConcurrentLinkedDeque;

import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.usfirst.frc.team948.pipeline.HSimpleEX;
import org.usfirst.frc.team948.pipeline.Pipe;
import org.usfirst.frc.team948.pipeline.SimpleEX;

import edu.wpi.cscore.CvSink;
import edu.wpi.cscore.CvSource;
import edu.wpi.first.wpilibj.CameraServer;
//import edu.wpi.first.wpilibj.Timer;
import java.util.Timer;
import java.util.TimerTask;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class visionProc implements VisionProccesor{
	private static final boolean bool = true;
	private static final double initialDistance = bool ? 34.7 : 34.7;
	private static final double initialHeight = bool ? 25.0 : 50.0;
	private static final double initialWidth = bool ? 10.0 : 21.0;
//	private static final double initialX = bool ? 39.5 : 0;
//	private static final double initialGamma = bool ? ((-5.0)*Math.PI)/180.0 : ((0.0)*Math.PI)/180.0;
	private threadOut gotten;
	private visionField lastOut;
	
	private Timer proccessingTimer;
	private CvSink cvSink;
	private CvSource vidOut;
	private Pipe pipeLine;
	private Mat mat;
	Thread processingThread;
	threadOut threadObjectData;
	
	public visionProc(){}
	
	public visionProc start(){
		gotten = new threadOut();
		lastOut = new visionField();
		threadObjectData = new threadOut();
		cvSink = CameraServer.getInstance().getVideo();
		vidOut = CameraServer.getInstance().putVideo("Processed", Robot.Camera_Width, Robot.Camera_Height);
		mat = new Mat();
		pipeLine = bool ? new SimpleEX() : new SimpleEX();
		proccessingTimer = new Timer();
		proccessingTimer.schedule(new TimerTask(){
			@Override
			public void run(){
				long start = System.currentTimeMillis();
				if (cvSink.grabFrame(mat) == 0) {
					vidOut.notifyError(cvSink.getError());
				}else{
					pipeLine.process(mat);
					ArrayList<MatOfPoint> cameraIn = pipeLine.findContoursOutput();
					int cont = cameraIn.size();
					int kprime = 0;
					int k = 0;
					double secondMaxSize = 0;
					double maxSize = 0;
					for(int i = 0; i < cont;i++){
						MatOfPoint temp0 = cameraIn.get(i);
						Rect temp1 = Imgproc.boundingRect(temp0);
						if(temp1.height*temp1.width >= maxSize){
							kprime = k;
							secondMaxSize = maxSize;
							k = i;
							maxSize = temp1.height*temp1.width;
						}
					}
					if(maxSize > 0){
						MatOfPoint l = cameraIn.get(k);
						Rect j = Imgproc.boundingRect(l);
						Imgproc.rectangle(mat, j.br(), j.tl(), new Scalar(255, 255, 255), 1);
						threadOut temp = new threadOut();
						boolean boool = false;
						temp.hasData = true;
						temp.area = l.size().area();
						temp.rectWidth = j.width;
						temp.rectHeight = j.height;
						temp.x = (j.tl().x + j.br().x)/2;
						temp.y = (j.tl().y + j.br().y)/2;
						temp.frameWidth = mat.width();
						if(secondMaxSize > 0){
							boool = true;
							MatOfPoint lprime = cameraIn.get(kprime);
							Rect jprime = Imgproc.boundingRect(lprime);
							threadOut nestedTemp = new threadOut();
							nestedTemp.hasData = true;
							nestedTemp.area = lprime.size().area();
							nestedTemp.rectWidth = jprime.width;
							nestedTemp.rectHeight = jprime.height;
							nestedTemp.x = (jprime.tl().x + jprime.br().x)/2;
							nestedTemp.y = (jprime.tl().y + jprime.br().y)/2;
							nestedTemp.frameWidth = mat.width();
							Imgproc.rectangle(mat, jprime.br(), jprime.tl(), new Scalar(0.0, 0.0, 255.0), 1);
							temp.secondValue = nestedTemp;
							temp.hasSecond = true;
						}
						long end = System.currentTimeMillis();
						long delta = end - start;
						if(boool)
							temp.secondValue.proccessTime = delta;
						temp.proccessTime = delta;
						setFrameData(temp);
					}
				}
				vidOut.putFrame(mat);
			}
		}, 0, 40);
		return this;
	}
	
	private double rectDistance(threadOut in){
		if(in.hasData){
			double H = in.rectHeight;
			return (initialHeight*initialDistance)/H;
		}
		System.out.println("preVNull:" + System.currentTimeMillis());
		return (Double) null;
	}
	
	private double getThetaSingleTape(threadOut in, boolean peg){
		if(in.hasData && peg){
			if(in.hasSecond){
				double W = in.rectWidth;
				double H = in.rectHeight;
				double uW = (H/initialHeight)*initialWidth;
				double correctedRatio = Math.min(1.0, W/uW);
				double theta = Math.acos(correctedRatio);
				SmartDashboard.putNumber("WOveruW", W/uW);
				SmartDashboard.putBoolean("inArcCosRange", Math.abs(W/uW) <= 1.0);
				theta = Math.copySign(theta,in.x - in.secondValue.x);
				return theta;
			}
		}else if(in.hasData){
			double W = in.rectWidth;
			double H = in.rectHeight;
			double uW = (H/initialHeight)*initialWidth;
			SmartDashboard.putNumber("WOveruW", W/uW);
			SmartDashboard.putBoolean("inArcCosRange", Math.abs(W/uW) <= 1.0);
			double correctedRatio = Math.min(1.0, W/uW);
			double theta = Math.acos(correctedRatio);
			return theta;
		}
		System.out.println("ThetaNull:" + System.currentTimeMillis());
		return (Double) null;
	}
	
	private double getCenterDistance(threadOut in, double theta, boolean peg){
		if(in.hasData && peg){
			if(in.hasSecond){
				double closestDistance = rectDistance(in);
				return closestDistance + (Math.tan(theta)*(2.0/2.0));
			}
		}else if(in.hasData){
			double closestDistance = rectDistance(in);
			double W = (initialWidth*initialDistance)/closestDistance;
			return closestDistance + (Math.tan(theta)*(W/2.0));
		}
		System.out.println("VNull:" + System.currentTimeMillis());
		return (Double) null;
	}
	
	private double getHeadingOffeset(threadOut in, double theta, boolean peg){
		if(in.hasData && peg){
			if(in.hasSecond){
				double x = (in.x + in.secondValue.x)/2.0;
				double wF = in.frameWidth;
				double epsilon = x - (wF/2.0);
//				double initialEpsilon = initialX - (wF/2.0);
//				2.0 is width in inches of tape
				double gamma = Math.atan((epsilon*2.0)/(initialDistance*initialWidth));
				return gamma;
			}
		}else if(in.hasData){
			double x = in.x;
			double wF = in.frameWidth;
			double epsilon = x - (wF/2.0);
//			double initialEpsilon = initialX - (wF/2.0);
//			2.0 is width in inches of tape
			double gamma = Math.atan((epsilon*2.0)/(initialDistance*initialWidth));
			return gamma;
		}
		System.out.println("GammaNull:" + System.currentTimeMillis());
		return (Double) null;
	}
	
	private double simpleHeading(threadOut in, boolean peg){
		if(in.hasData && peg){
			if(in.hasSecond){
				double x = (in.x + in.secondValue.x)/2.0;
				double wF = in.frameWidth;
				double epsilon = x - (wF/2.0);
				double zeta = epsilon/(wF/2.0);
				return zeta;
			}
		}else if(in.hasData){
			double x = in.x;
			double wF = in.frameWidth;
			double epsilon = x - (wF/2.0);
			double zeta = epsilon/(wF/2.0);
			return zeta;
		}
		System.out.println("ZetaNull:" + System.currentTimeMillis());
		return (Double) null;
	}
	
	private double getOmega(threadOut in, double centerDistance, boolean peg){
		if(in.hasData && peg){
			if(in.hasSecond){
				double x = (in.x + in.secondValue.x)/2.0;
				double wF = in.frameWidth;
				double epsilon = x - (wF/2);
//				double initialEpsilon = initialX - (wF/2);
//				2.0 is width in inches of tape
				double omega = (centerDistance*epsilon*2.0)/(initialDistance*initialWidth);
				return omega;
			}
		}else if(in.hasData){
			double x = in.x;
			double wF = in.frameWidth;
			double epsilon = x - (wF/2);
//			double initialEpsilon = initialX - (wF/2);
//			2.0 is width in inches of tape
			double omega = (centerDistance*epsilon*2.0)/(initialDistance*initialWidth);
			return omega;
		}
		System.out.println("OmegaNull:" + System.currentTimeMillis());
		return (Double) null;
	}
	
	public boolean dataExists(){
		threadOut temp = getFrameData();
		if(temp.hasData){
			gotten = temp;
			SmartDashboard.putNumber("visionArea1",temp.area);
			SmartDashboard.putNumber("visionFrameWidth1",temp.frameWidth);
			SmartDashboard.putNumber("visionRectHeight1",temp.rectHeight);
			SmartDashboard.putNumber("visionRectWidth1",temp.rectWidth);
			SmartDashboard.putNumber("visionX1",temp.x);
			SmartDashboard.putNumber("visionY1",temp.y);
			SmartDashboard.putNumber("visionProccessTime1",temp.proccessTime);
			SmartDashboard.putBoolean("hasSecond", temp.hasSecond);
			if(temp.hasSecond){
				SmartDashboard.putNumber("visionArea2",temp.area);
				SmartDashboard.putNumber("visionFrameWidth2",temp.frameWidth);
				SmartDashboard.putNumber("visionfRectHeight2",temp.rectHeight);
				SmartDashboard.putNumber("visionRectWidth2",temp.rectWidth);
				SmartDashboard.putNumber("visionX2",temp.x);
				SmartDashboard.putNumber("visionY2",temp.y);
				SmartDashboard.putNumber("visionProccessTime2",temp.proccessTime);
			}
			return true;
		}
		return false;
	}
	
	public visionField getData(){
		if(gotten.hasData){
			if(gotten.hasSecond){
				visionField out = new visionField();
				out.theta = getThetaSingleTape(gotten, true);
				out.v = getCenterDistance(gotten, Math.abs(out.theta), true);
				out.zeta = simpleHeading(gotten, true);
				out.omega = getOmega(gotten, out.v, true);
				out.gamma = getHeadingOffeset(gotten, Math.abs(out.theta), true);
				out.isTape = false;
				lastOut = out;
				return out;
			}else{
				visionField out = new visionField();
				out.theta = getThetaSingleTape(gotten,false);
				out.v = getCenterDistance(gotten, out.theta,false);
				out.zeta = simpleHeading(gotten,false);
				out.omega = getOmega(gotten, out.v,false);
				out.gamma = getHeadingOffeset(gotten, out.theta,false);
				out.isTape = true;
				lastOut = out;
				return out;
			}
		}else if(!lastOut.equals(new threadOut())){
			return lastOut;
		}
		return new visionField();
	}
	
	public synchronized void setFrameData(threadOut in){
		threadObjectData = in;
	}
	
	public synchronized threadOut getFrameData(){
		return threadObjectData;
	}
	
	private class threadOut{
		public double area;
		public double rectWidth;
		public double rectHeight;
		public double frameWidth;
		public double x;
		public double y;
		public threadOut secondValue;
		public long proccessTime;
		public boolean hasSecond = false;
		public boolean hasData = false;
	}
}