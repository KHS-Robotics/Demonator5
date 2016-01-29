package org.usfirst.frc.team4342.api.multithreading;

public abstract class Component implements Runnable
{
	public static final int SLEEP_MILLIS = 20;
	protected boolean run;
	
	public boolean isRunning()
	{
		return this.run;
	}
}
