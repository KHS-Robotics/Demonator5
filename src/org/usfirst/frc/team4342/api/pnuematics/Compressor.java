package org.usfirst.frc.team4342.api.pnuematics;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Relay;

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
	
	public Relay getRelay()
	{
		return relay;
	}
	
	public DigitalInput getPressureSwitch()
	{
		return pSwitch;
	}
}
