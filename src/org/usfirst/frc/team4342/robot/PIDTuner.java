package org.usfirst.frc.team4342.robot;

import org.usfirst.frc.team4342.robot.components.Repository;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * Temporary class to tune PID values.
 */
public final class PIDTuner 
{
	private PIDTuner() {}
	
	private static boolean run;
	
	public static void startUpdating()
	{
		if(run)
			return;
		
		run = true;
		new PIDUpdater().start();
	}
	
	public static void stopUpdating()
	{
		run = false;
	}
	
	private static class PIDUpdater extends Thread implements Runnable
	{
		@Override
		public void run()
		{
			while(run)
			{
				try
				{
					
					Thread.sleep(20);
				}
				catch(Exception ex)
				{
					Repository.Logs.error("Failed to update PID values!", ex);
					break;
				}
			}
		}
	}
}
