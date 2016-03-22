package org.usfirst.frc.team4342.robot.multithreading;

public abstract class Component implements Runnable
{
	public static final int SLEEP_MILLIS = 20;
	protected boolean run;
	
	public abstract void stopAll();
	
	public boolean isRunning()
	{
		return this.run;
	}
}
