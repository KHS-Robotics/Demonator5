package org.usfirst.frc.team4342.api.pnuematics;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.Relay;
import edu.wpi.first.wpilibj.Relay.Value;

public class PneumaticsController 
{
	private Relay compressor;
	private DoubleSolenoid solenoid;
	private boolean compressorEnable;
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
	
	public void turnOnCompressor()
	{
		compressor.set(Value.kOn);
		
		compressorEnable = true;
	}
	
	public void turnOffCompressor()
	{
		compressor.set(Value.kOff);
		
		compressorEnable = false;
	}
	
	public boolean isCompressorEnabled()
	{
		return (compressorEnable);
	}
	
	public boolean isCompressorDisabled()
	{
		return !(compressorEnable);
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
