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
						SmartDashboard.getNumber("Drive-Yaw-I"), 
						SmartDashboard.getNumber("Drive-Yaw-D")
					);
					
					Repository.TankDrive.setRightPID(
						SmartDashboard.getNumber("Drive-Right-P"), 
						SmartDashboard.getNumber("Drive-Right-I"), 
						SmartDashboard.getNumber("Drive-Right-D")
					);
					
					Repository.TankDrive.setLeftPID(
						SmartDashboard.getNumber("Drive-Left-P"), 
						SmartDashboard.getNumber("Drive-Left-I"), 
						SmartDashboard.getNumber("Drive-Left-D")
					);
					
					Repository.ShooterController.setPID(
						SmartDashboard.getNumber("Shooter-P"), 
						SmartDashboard.getNumber("Shooter-I"), 
						SmartDashboard.getNumber("Shooter-D")
					);
					
					Repository.ArmController.getPIDController().setPIDUp(
						SmartDashboard.getNumber("Arm-P-Up"), 
						SmartDashboard.getNumber("Arm-I-Up"), 
						SmartDashboard.getNumber("Arm-D-Up")
					);
					
					Repository.ArmController.getPIDController().setPIDDown(
						SmartDashboard.getNumber("Arm-P-Down"), 
						SmartDashboard.getNumber("Arm-I-Down"), 
						SmartDashboard.getNumber("Arm-D-Down")
					);
					
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
