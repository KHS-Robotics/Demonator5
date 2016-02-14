package org.usfirst.frc.team4342.api.vision;

public class Goal
{
	private final double[] lowBound, highBound;
	
	public Goal(double[] lowBound, double[] highBound)
	{
		this.lowBound = lowBound;
		this.highBound = highBound;
	}
	
	public double[] getLowBound()
	{
		return lowBound;
	}
	
	public double[] getHighBound()
	{
		return highBound;
	}
}