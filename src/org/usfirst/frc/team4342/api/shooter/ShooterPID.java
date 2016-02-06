package org.usfirst.frc.team4342.api.shooter;

public final class ShooterPID 
{
	private ShooterPID() {}
	
	// Values for going up (higher to fight gravity)
	public static final double kP = 0.007;
	public static final double kI = 0.0;
	public static final double kD = 0.10;
	
	// Values for going down (lower because of gravity)
	public static final double kPd = 0.001;
	public static final double kId = 0.0;
	public static final double kDd = 0.008;
}