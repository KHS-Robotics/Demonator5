 package org.usfirst.frc.team4342.api.shooter;

import org.usfirst.frc.team4342.api.logging.ExceptionInfo;
import org.usfirst.frc.team4342.robot.components.Repository;

import edu.wpi.first.wpilibj.CANTalon;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.Solenoid;

public class Shooter 
{
	private static boolean run;
	
	public static void startAutomaticMode(Joystick j, CANTalon rightMotor, CANTalon leftMotor, Solenoid cylinder, int extendButton)
	{
		if(run)
			return;
		
		run = true;
		
		Thread t = new Thread(new Runnable()
		{
			@Override
			public void run()
			{
				while(run)
				{
					try
					{
						if(DriverStation.getInstance().isEnabled() && DriverStation.getInstance().isOperatorControl())
						{
							
						}
						
						Thread.sleep(50);
					}
					catch(Exception ex)
					{
						Repository.Logs.error(
							"Unexpected error in shooter (" + ExceptionInfo.getType(ex) + ")", 
							ex
						);
					}
				}
			}
		});
		
		t.start();
	}
	
	public static void stopAutomaticMode()
	{
		run = false;
	}
}
