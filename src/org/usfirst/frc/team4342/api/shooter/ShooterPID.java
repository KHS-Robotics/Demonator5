package org.usfirst.frc.team4342.api.shooter;

public final class ShooterPID 
{
	private ShooterPID() {}
	
	public static final double kP = 0.05;
	public static final double kI = 0.0 / 1000.0;
	public static final double kD = 0.04;
	public static final double kF = 0.01;
}