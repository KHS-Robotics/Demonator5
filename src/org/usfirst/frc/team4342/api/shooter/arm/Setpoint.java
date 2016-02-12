package org.usfirst.frc.team4342.api.shooter.arm;

public class Setpoint 
{	
	private final int button;
	private final int encoderCounts;
	
	/**
	 * Constructs an elevator setpoint
	 * @param button the button associated with the setpoint
	 * @param encoderCounts the encoder counts for the setpoint
	 */
	public Setpoint(int button, int encoderCounts) 
	{
		this.button = button;
		this.encoderCounts = encoderCounts;
	}
	
	public int getButton() 
	{
		return button;
	}
	
	public int getEncoderCounts() 
	{
		return encoderCounts;
	}
}
