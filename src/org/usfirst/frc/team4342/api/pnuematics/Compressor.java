package org.usfirst.frc.team4342.api.pnuematics;

import org.usfirst.frc.team4342.api.logging.ExceptionInfo;
import org.usfirst.frc.team4342.robot.components.Repository;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Relay;
import edu.wpi.first.wpilibj.Relay.Value;

/**
 * This class is for controlling a compressor
 */
public class Compressor 
{
	private static boolean enabled;
	private static boolean run;
	
	/**
	 * Spawns a thread that automatically checks if the compressor should be on
	 */
	public static void startAutomaticMode(Relay relay, DigitalInput pSwitch)
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
						double velocity = Math.sqrt(Math.pow(Repository.Navx.getVelocityX(), 2) + Math.pow(Repository.Navx.getVelocityY(), 2));
						double throttle = Math.sqrt(Math.pow(Repository.DriveStick.getX(), 2) + Math.pow(Repository.DriveStick.getY(), 2));
						double ratio = (velocity/throttle);
						double PUSH_VALUE = 20;
						
						if(pSwitch.get() || (ratio < PUSH_VALUE)
						{
							relay.set(Value.kOff);
							enabled = false;
						}
						else
						{
							relay.set(Value.kForward);
							enabled = true;
						}
						
						Thread.sleep(100);
					}
					catch(Exception ex)
					{
						Repository.Logs.error(
							"Unexpected error with compressor (" + ExceptionInfo.getType(ex) + ")", 
							ex
						);
						
						break;
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
	
	/**
	 * Determine if the compressor is disabled
	 * @return true if the compressor is off, false otherwise
	 */
	public synchronized boolean isDisabled()
	{
		return !enabled;
	}
	
	/**
	 * Determine if the compressor is enabled
	 * @return true if the compressor is on, false otherwise
	 */
	public synchronized boolean isEnabled()
	{
		return enabled;
	}
}
