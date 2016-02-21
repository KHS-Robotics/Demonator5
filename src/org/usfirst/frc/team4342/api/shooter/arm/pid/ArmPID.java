package org.usfirst.frc.team4342.api.shooter.arm.pid;

public final class ArmPID 
{
	private ArmPID() {}
	
	// Up values
	public static final double kP = 0.12;
	public static final double kI = 0.0;
	public static final double kD = 0.04;
	
	// Down values
	public static final double kPd = 0.001;
	public static final double kId = 0.0 / 1000.0;
	public static final double kDd = 0.004;
}
