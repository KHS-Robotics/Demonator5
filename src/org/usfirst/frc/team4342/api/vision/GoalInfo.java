package org.usfirst.frc.team4342.api.vision;

public class GoalInfo 
{
	private final double distance, angle;
	
	public GoalInfo(double distance, double angle)
	{
		this.distance = distance;
		this.angle = angle;
	}
	
	public double getDistance()
	{
		return distance;
	}
	
	public double getAngle()
	{
		return angle;
	}
}
