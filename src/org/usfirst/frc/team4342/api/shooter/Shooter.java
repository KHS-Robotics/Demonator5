package org.usfirst.frc.team4342.api.shooter;

import edu.wpi.first.wpilibj.CANTalon;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.Solenoid;

/**
 * Shooter DTO. I hate how Java doesn't have properties like C#...
 */
public class Shooter 
{
	private Joystick j;
	private CANTalon accumulator, rightMotor, leftMotor;
	private Solenoid loaderX, loaderY;
	
	public Shooter(Joystick j, CANTalon accumulator, CANTalon rightMotor, 
							  CANTalon leftMotor, Solenoid loaderX, Solenoid loaderY)
	{
		this.j = j;
		this.accumulator = accumulator;
		this.rightMotor = rightMotor;
		this.leftMotor = leftMotor;
		this.loaderX = loaderX;
		this.loaderY = loaderY;
	}
	
	public Joystick getJoystick()
	{
		return j;
	}
	
	public CANTalon getAccumulator()
	{
		return accumulator;
	}
	
	public CANTalon getRightMotor()
	{
		return rightMotor;
	}
	
	public CANTalon getLeftMotor()
	{
		return leftMotor;
	}
	
	public Solenoid getLoaderX()
	{
		return loaderX;
	}
	
	public Solenoid getLoaderY()
	{
		return loaderY;
	}
}
