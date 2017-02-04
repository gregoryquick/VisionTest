package org.usfirst.frc.team948.robot;

import edu.wpi.first.wpilibj.IterativeRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
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
		proccesor = new visionProc();
		double[] input = {(Double) null,(Double) null,(Double) null};
		try {
			input = proccesor.objectAverage();
		} catch (InterruptedException e) {
			e.printStackTrace();
		}
		SmartDashboard.putNumber("averageX", input[0]);
		SmartDashboard.putNumber("averageY", input[1]);
		SmartDashboard.putNumber("averageArea", input[2]);
		}
	
	public void loop(double prevTime){
		while(prevTime+tickDistance <= clock.get()){continue;};
		double[] input = {(Double) null,(Double) null,(Double) null};
		try {
			input = proccesor.objectAverage();
		} catch (InterruptedException e) {
			e.printStackTrace();
		}
		SmartDashboard.putNumber("averageX", input[0]);
		SmartDashboard.putNumber("averageY", input[1]);
		SmartDashboard.putNumber("averageArea", input[2]);
	}
}
