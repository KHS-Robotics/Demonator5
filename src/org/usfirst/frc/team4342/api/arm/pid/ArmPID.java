package org.usfirst.frc.team4342.api.arm.pid;

public final class ArmPID 
{
	private ArmPID() {}
	
	// Up values
	public static final double kP = 0.058;
	public static final double kI = 0.0;
	public static final double kD = 0.005;
	
	// Down values
	public static final double kPd = 0.058 / 100.0;
	public static final double kId = 0.0 / 1000.0;
	public static final double kDd = 0.005 / 100.0;
}
