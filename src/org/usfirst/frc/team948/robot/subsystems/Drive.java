package org.usfirst.frc.team948.robot.subsystems;

import org.usfirst.frc.team948.robot.Robot;
import org.usfirst.frc.team948.robot.RobotMap;
import org.usfirst.frc.team948.robot.commands.ManualDrive;
import org.usfirst.frc.team948.utilities.MathUtilities;
import org.usfirst.frc.team948.utilities.PreferenceKeys;

import edu.wpi.first.wpilibj.PIDController;
import edu.wpi.first.wpilibj.PIDOutput;
import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 *
 */
public class Drive extends Subsystem implements PIDOutput {

	public enum Direction {
		FORWARD, BACKWARD
	}

	private PIDController drivePID;
	private volatile double PIDOutput;
	private double prevTurnError;
	private int cyclesOnTarget;
	private double autonomousHeading = 0;

	private static final double PID_MIN_OUTPUT = 0.05;
	private static final double PID_MAX_OUTPUT = 0.5;
	private static final double SLOW_DOWN_ERROR = 10.0;
	private static final double MIN_POWER_TURN = 0.15;

	private static final double DRIVE_STRAIGHT_ON_HEADING_P = 0.025;
	private static final double DRIVE_STRAIGHT_ON_HEADING_I = 0.001;
	private static final double DRIVE_STRAIGHT_ON_HEADING_D = 0.03;
	private static final double DRIVE_STRAIGHT_ON_HEADING_F = 0.0;

	private static final double DEFAULT_TICKS_PER_INCH = 121;
	private static final double DEFAULT_DISTANCE_TOLERANCE = 1.0 * DEFAULT_TICKS_PER_INCH;

	private double kp;
	private double ki;
	private double kd;
	private double kf;
	private double turnError;
	private double turnTolerance;
	private int requiredCyclesOnTarget;

	public void initDefaultCommand() {
		setDefaultCommand(new ManualDrive());
	}

	@Override
	public void pidWrite(double output) {
		PIDOutput = output;
	}

	public void drivePIDInit(double p, double i, double d, double setPoint, double tolerance,
			int toleranceBuffLength) {
		drivePID = new PIDController(p, i, d, RobotMap.navX, this);
		drivePID.reset();
		drivePID.setOutputRange(-0.3, 0.3);
		if (tolerance != 0.0) {
			drivePID.setAbsoluteTolerance(tolerance);
			drivePID.setToleranceBuffer(toleranceBuffLength);
		}
		drivePID.setSetpoint(setPoint);
		PIDOutput = 0;
		drivePID.enable();
		SmartDashboard.putString("Current PID kp, ki, kd, kf",
				drivePID.getP() + ", " + drivePID.getI() + ", " + drivePID.getD() + ", " + drivePID.getF());
		SmartDashboard.putNumber("Current PID setpoint", drivePID.getSetpoint());
	}

	public void driveOnHeadingInit(double desiredHeading) {
		kp = RobotMap.preferences.getDouble("DRIVE_STRAIGHT_ON_HEADING_P", DRIVE_STRAIGHT_ON_HEADING_P);
		ki = RobotMap.preferences.getDouble("DRIVE_STRAIGHT_ON_HEADING_I", DRIVE_STRAIGHT_ON_HEADING_I);
		kd = RobotMap.preferences.getDouble("DRIVE_STRAIGHT_ON_HEADING_D", DRIVE_STRAIGHT_ON_HEADING_D);
		kf = RobotMap.preferences.getDouble("DRIVE_STRAIGHT_ON_HEADING_F", DRIVE_STRAIGHT_ON_HEADING_F);
		drivePIDInit(kp, ki, kd, desiredHeading, 0, 0);

	}

	public void driveOnHeading(double power, double desiredHeading) {
		drivePID.setSetpoint(desiredHeading);
//		setAutonomousHeading(desiredHeading);
		double error = drivePID.getError();
//		double outputRange = MathUtilities.clamp(
//				PID_MIN_OUTPUT + (Math.abs(error) / 15.0) * (PID_MAX_OUTPUT - PID_MIN_OUTPUT), 0, PID_MAX_OUTPUT);
//		drivePID.setOutputRange(-outputRange, outputRange);

		double currentPIDOutput = MathUtilities.clamp(PIDOutput+kf, -0.5, 0.5);

		SmartDashboard.putNumber("driveOnHeading PID error", error);
		SmartDashboard.putNumber("driveOnHeading PID output", currentPIDOutput);
		SmartDashboard.putNumber("driveOnHeading rawPower", power);
		SmartDashboard.putNumber("desired heading", desiredHeading);

		double pL = power;
		double pR = power;
		pL += currentPIDOutput;
		pR -= currentPIDOutput;
		SmartDashboard.putNumber("Left driveOnHeading power", pL);
		SmartDashboard.putNumber("Right driveOnHeading power", pR);
		tankDrive(pL, pR);
	}

	public void driveOnHeadingEnd() {
		drivePID.reset();
		drivePID.free();
		PIDOutput = 0;
		stop();
	}

	public void tankDrive(double leftPower, double rightPower) {
//		RobotMap.frontLeftMotor.set(Math.abs(leftPower) > 0.5 ? Math.copySign(0.5, leftPower) : leftPower);
//		RobotMap.backLeftMotor.set(Math.abs(leftPower) > 0.5 ? Math.copySign(0.5, leftPower) : leftPower);
//		RobotMap.frontRightMotor.set(Math.abs(rightPower) > 0.5 ? Math.copySign(0.5, rightPower) : rightPower);
//		RobotMap.backRightMotor.set(Math.abs(rightPower) > 0.5 ? Math.copySign(0.5, rightPower) : rightPower);
	}

	public void stop() {
		RobotMap.frontLeftMotor.disable();
		RobotMap.backLeftMotor.disable();
		RobotMap.frontRightMotor.disable();
		RobotMap.backRightMotor.disable();
	}

	public void turnToHeadingInitNoPID(double desiredHeading) {
		setAutonomousHeading(desiredHeading);
		turnError = desiredHeading - RobotMap.navX.getAngle();
		requiredCyclesOnTarget = RobotMap.preferences.getInt("TURN_TOLERANCE_BUFFER", 6);
		turnTolerance = RobotMap.preferences.getDouble("TURN_TOLERANCE", 1.0);
		kp = RobotMap.preferences.getDouble("TURN_P", .025);
		SmartDashboard.putNumber("desired heading", desiredHeading);
		cyclesOnTarget = 0;
		prevTurnError = 0;
	}

	public void turnToHeadingNoPID(double maxPower) {
		turnError = getAutonomousHeading() - RobotMap.navX.getAngle();
		double angularVel = prevTurnError - turnError;
		double predictedErrorNextCycle = turnError - angularVel;
		double adjustedPower = MathUtilities.clamp(Math.abs(predictedErrorNextCycle * kp), MIN_POWER_TURN, maxPower);
		SmartDashboard.putNumber("turnToHeadingNoPID turnError", turnError);
		SmartDashboard.putNumber("turnToHeadingNoPID scaledPower", adjustedPower);
		adjustedPower = Math.copySign(adjustedPower, predictedErrorNextCycle);

		// shut power off if current or predicted error within tolerance
		if (Math.abs(turnError) <= turnTolerance) {
			cyclesOnTarget++;
			adjustedPower = 0;
		} else {
			cyclesOnTarget = 0;
		}

		if (Math.abs(predictedErrorNextCycle) <= turnTolerance) {
			adjustedPower = 0;
		}

		tankDrive(adjustedPower, -adjustedPower);
		prevTurnError = turnError;
	}

	public void turnToHeadingEndNoPID(double newHeading) {
		stop();
	}

	public boolean isOnHeadingNoPID() {
		return cyclesOnTarget >= requiredCyclesOnTarget;
	}

	public double getTicksFromInches(double inches) {
		return inches * RobotMap.preferences.getDouble("TICKS_PER_INCH", DEFAULT_TICKS_PER_INCH);
	}

	public double getTicksPerInch() {
		return RobotMap.preferences.getDouble("TICKS_PER_INCH", DEFAULT_TICKS_PER_INCH);
	}

	public double getDistanceToleranceInTicks() {
		return RobotMap.preferences.getDouble("DISTANCE_TOLERANCE_IN_TICKS",
				DEFAULT_DISTANCE_TOLERANCE);
	}

	public double getAutonomousHeading() {
		return autonomousHeading;
	}

	public void setAutonomousHeading(double autonomousHeading) {
		this.autonomousHeading = autonomousHeading;
	}

}