package org.usfirst.frc.team4342.api.drive.pid;

public final class DrivePID 
{
	private DrivePID() {}
	
	public static class Rotational
	{
		public static final double kP = 0.04;
		public static final double kI = 0.0 / 1000.0;
		public static final double kD = 0.03;
	}
}
