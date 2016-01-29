package org.usfirst.frc.team4342.api.pnuematics;

import org.usfirst.frc.team4342.robot.components.Repository;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Relay;
import edu.wpi.first.wpilibj.Relay.Value;

/**
 * This class is for controlling a compressor
 */
public class Compressor 
{
	private Relay relay;
	private DigitalInput pSwitch;
	
	public Compressor(Relay relay, DigitalInput pSwitch)
	{
		this.relay = relay;
		this.pSwitch = pSwitch;
	}
	
	public void handle()
	{
		final double velocity = Math.sqrt(Math.pow(Repository.Navx.getVelocityX(), 2) + Math.pow(Repository.Navx.getVelocityY(), 2));
		final double throttle = Math.sqrt(Math.pow(Repository.DriveStick.getX(), 2) + Math.pow(Repository.DriveStick.getY(), 2));
		final double ratio = (velocity/throttle);
		final double PushValue = 20.0;
		
		if(pSwitch.get() || (ratio < PushValue))
		{
			relay.set(Value.kOff);
		}
		else
		{
			relay.set(Value.kForward);
		}
	}
	
	public Relay getRelay()
	{
		return relay;
	}
	
	public DigitalInput getPressureSwitch()
	{
		return pSwitch;
	}
}
