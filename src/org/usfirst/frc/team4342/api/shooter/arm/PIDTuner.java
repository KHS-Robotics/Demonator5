package org.usfirst.frc.team4342.api.shooter.arm;

import org.usfirst.frc.team4342.robot.Repository;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * Temporary class to tune PID values.
 */
public final class PIDTuner 
{
	private PIDTuner() {}
	
	private static boolean run;
	private static int setPIDButton;
	
	public static void startUpdating(int setPIDButton)
	{
		if(run)
			return;
		
		run = true;
		
		PIDTuner.setPIDButton = setPIDButton;
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
					if(Repository.SwitchBox.getRawButton(setPIDButton))
					{
						Repository.ArmController.setPID(
							-SmartDashboard.getNumber("Arm-P-Up") / 100.0,
							-SmartDashboard.getNumber("Arm-I-Up") / 1000.0,
							-SmartDashboard.getNumber("Arm-D-Up") / 100.0,
							-SmartDashboard.getNumber("Arm-P-Up") / 100.0,
							-SmartDashboard.getNumber("Arm-I-Up") / 1000.0,
							-SmartDashboard.getNumber("Arm-D-Up") / 100.0
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
