package org.usfirst.frc.team4342.api.shooter.arm.pid;

public final class ArmPID 
{
	private ArmPID() {}
	
	// Up values
	public static final double kP = 0.05;
	public static final double kI = 0.008 / 1000.0;
	public static final double kD = 0.03;
	
	// Down values
	public static final double kPd = 0.005;
	public static final double kId = 0.001 / 1000.0;
	public static final double kDd = 0.003;
}
