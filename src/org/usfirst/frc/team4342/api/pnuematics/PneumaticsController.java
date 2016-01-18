package org.usfirst.frc.team4342.api.pnuematics;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.Relay;
import edu.wpi.first.wpilibj.Relay.Value;

public class PneumaticsController 
{
	private Relay compressor;
	private DoubleSolenoid solenoid;
	private boolean compressorEnabled;
	private boolean isLowGear;
	
	
	public PneumaticsController(Relay compressor, DoubleSolenoid solenoid)
	{
		this.compressor = compressor;
		this.solenoid = solenoid;
	}
	
	public void shiftToLowGear()
	{
		solenoid.set(DoubleSolenoid.Value.kReverse);
		
		isLowGear = true;
	}
	
	public void shiftToHighGear()
	{
		solenoid.set(DoubleSolenoid.Value.kForward);
		
		isLowGear = false;
	}
	
	public void enableCompressor()
	{
		compressor.set(Value.kOn);
		
		compressorEnabled = true;
	}
	
	public void disableCompessor()
	{
		compressor.set(Value.kOff);
		
		compressorEnabled = false;
	}
	
	public boolean isCompressorEnabled()
	{
		return (compressorEnabled);
	}
	
	public boolean isCompressorDisabled()
	{
		return !(compressorEnabled);
	}
	
	public boolean isLowGear()
	{
		return (isLowGear);
	}
	
	public boolean isHighGear()
	{
		return !(isLowGear);
	}
}
