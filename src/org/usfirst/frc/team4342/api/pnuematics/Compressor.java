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
	private volatile Relay relay;
	private volatile DigitalInput pSwitch;
	
	private volatile boolean enabled;
	
	/**
	 * Constructs a normal compressor with a relay and digital pressure switch
	 * @param relay the relay that turns the compressor on and off
	 * @param pSwitch the digital sensor that determines if the air tank is full
	 */
	public Compressor(Relay relay, DigitalInput pSwitch)
	{
		this.relay = relay;
		this.pSwitch = pSwitch;
	}
	
	/**
	 * Spawns a thread that automatically checks if the compressor should be on
	 */
	public void setAutomaticMode()
	{
		Thread t = new Thread(new Runnable() 
		{
			@Override
			public void run()
			{
				while(true)
				{
					try
					{
						if(pSwitch.get())
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
	
	/**
	 * Turns off the compressor
	 */
	public synchronized void disable()
	{
		relay.set(Value.kOff);
		enabled = false;
	}
	
	/**
	 * Turns on the compressor
	 */
	public synchronized void enable()
	{
		relay.set(Value.kForward);
		enabled = true;
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
