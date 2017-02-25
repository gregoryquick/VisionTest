package org.usfirst.frc.team948.utilities;

import org.usfirst.frc.team948.robot.RobotMap;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.PIDSource;
import edu.wpi.first.wpilibj.PIDSourceType;

public class MeshedEncoders implements PIDSource{
	private static final double leftTicksPerInch = 1931.0/99.5;
	private static final double rightTicksPerInch = 1892.0/99.5;
	private final Encoder leftEncoder;
	private final Encoder rightEncoder;
	
	public MeshedEncoders(RobotMap robotMap){
		leftEncoder = robotMap.leftEncoder;
		rightEncoder = robotMap.rightEncoder;
	}
	
	public double rightEncoderTicks(){
		return rightEncoder.get();
	}
	
	public double rightEncoderInches(){
		double out = ( (double) rightEncoder.get())/rightTicksPerInch;
		return out;
	}
	
	public double leftEncoderTicks(){
		return leftEncoder.get();
	}
	
	public double leftEncoderInches(){
		double out = ((double) leftEncoder.get())/leftTicksPerInch;
		return out;
	}
	
	public double meshedEncoderInches(){
		return (leftEncoderInches() + rightEncoderInches())/2.0;
	}

	public double meshedEncoderTicks(){
		return (meshedEncoderInches()*(leftTicksPerInch + rightTicksPerInch))/2.0;
	}
	
	public double MaxEncoderTicks(){
		return rightEncoderTicks() > leftEncoderTicks() ? rightEncoderTicks() : leftEncoderTicks();
	}
	
	public double MaxEncoderInches(){
		return rightEncoderInches() > leftEncoderInches() ? rightEncoderInches() : leftEncoderInches();
	}
	
	@Override
	public void setPIDSourceType(PIDSourceType pidSource) {
		// TODO Auto-generated method stub
		
	}

	@Override
	public PIDSourceType getPIDSourceType() {
		// TODO Auto-generated method stub
		return null;
	}

	@Override
	public double pidGet() {
		return meshedEncoderInches();
	}

}
