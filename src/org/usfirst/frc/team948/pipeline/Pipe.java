package org.usfirst.frc.team948.pipeline;

import java.util.ArrayList;

import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;

public interface Pipe {
	public ArrayList<MatOfPoint> findContoursOutput();
	public void process(Mat a);
}
