package org.usfirst.frc.team4342.api.shooter.pid;

public final class ShooterPID 
{
	private ShooterPID() {}
	
	public static final double kP = 0.001;
	public static final double kI = 0.0 / 1000.0;
	public static final double kD = 0.5;
	public static final double kF = 0.01;
}