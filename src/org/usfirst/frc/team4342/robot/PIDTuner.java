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
					Repository.TankDrive.setYawPID(
						SmartDashboard.getNumber("Drive-Yaw-P"), 
						SmartDashboard.getNumber("Drive-Yaw-I") / 1000.0, 
						SmartDashboard.getNumber("Drive-Yaw-D")
					);
					
//					Repository.ShooterController.setPID(
//						SmartDashboard.getNumber("Shooter-P"), 
//						SmartDashboard.getNumber("Shooter-I") / 1000.0, 
//						SmartDashboard.getNumber("Shooter-D")
//					);
					
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
