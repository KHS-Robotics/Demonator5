package org.usfirst.frc.team4342.robot.api.pdp;

import edu.wpi.first.wpilibj.PowerDistributionPanel;

public class PdpInfoExtractor 
{
	private PowerDistributionPanel pdp;
	
	public PdpInfoExtractor()
	{
		this.pdp = new PowerDistributionPanel();
	}
	
	public double getCurrent(int channel)
	{
		return pdp.getCurrent(channel);
	}
	
	public double getAverageCurrent()
	{
		return pdp.getTotalCurrent() / 16;
	}
	
	public double getTotalCurrent()
	{
		return pdp.getTotalCurrent();
	}
	
	public double getTotalEnergy()
	{
		return pdp.getTotalEnergy();
	}
	
	public double getTotalPower()
	{
		return pdp.getTotalPower();
	}
	
	public double getVoltage()
	{
		return pdp.getVoltage();
	}
	
	public double getTemperature()
	{
		return pdp.getTemperature();
	}
	
	public void clearStickyFaults()
	{
		pdp.clearStickyFaults();
	}
	
	public PowerDistributionPanel getPdp()
	{
		return pdp;
	}
}
