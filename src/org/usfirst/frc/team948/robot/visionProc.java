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
				for(int i = 0; i < cont;i++){
					double[] properties = new double[3];
					MatOfPoint temp0 = cameraIn.get(i);
					properties[2] = temp0.size().area();
					Rect temp1 = Imgproc.boundingRect(temp0);
					properties[0] = (temp1.tl().x +temp1.br().x)/2;
					properties[1] = (temp1.tl().y +temp1.br().y)/2;
					output.offerFirst(properties);
					Imgproc.rectangle(mat, temp1.br(), temp1.tl(),
							new Scalar(255, 255, 255), 1);
				}
				out.putFrame(mat);
				objects.addFirst(output);
			}
		});
		processingThread.setDaemon(true);
		processingThread.start();
	}
}