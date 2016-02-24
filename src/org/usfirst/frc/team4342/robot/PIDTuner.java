package org.usfirst.frc.team4342.robot;

import org.usfirst.frc.team4342.api.arm.pid.ArmPID;
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
					if(Repository.SwitchBox.getRawButton(7))
					{
						Repository.ArmController.setPID(
							SmartDashboard.getNumber("Arm-P-Up") / 100.0,
							SmartDashboard.getNumber("Arm-I-Up") / 1000.0,
							SmartDashboard.getNumber("Arm-D-Up") / 100.0,
							SmartDashboard.getNumber("Arm-P-Down") / 100.0,
							SmartDashboard.getNumber("Arm-I-Down") / 1000.0,
							SmartDashboard.getNumber("Arm-D-Down") / 100.0
						);
						
						SmartDashboard.putBoolean("PID-IsUpdating", true);
					}
					else
					{
						Repository.ArmController.setPID(ArmPID.kP, ArmPID.kI, ArmPID.kD, ArmPID.kPd, ArmPID.kId, ArmPID.kDd);
						SmartDashboard.putBoolean("PID-IsUpdating", false);
					}
					
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
