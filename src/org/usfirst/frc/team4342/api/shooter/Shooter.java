package org.usfirst.frc.team4342.api.shooter;

import edu.wpi.first.wpilibj.CANTalon;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.Solenoid;

/**
 * Shooter DTO. I hate how Java doesn't have properties like C#...
 */
public class Shooter 
{
	public static final int MIN_ENC_VELOCITY = 30;
	
	private Joystick j;
	private CANTalon accumulator, rightMotor, leftMotor;
	private Solenoid loaderX, loaderY;
	
	private ShooterState state;
	
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
	
	public void handle()
	{
		state = loaderX.get() ? ShooterState.FIRED : ShooterState.LOADED;
		
		if (state == ShooterState.LOADED)
		{
			if (j.getRawButton(1))
			{
				loaderY.set(true);
				rightMotor.set(1);
				leftMotor.set(1);
				
				if (j.getRawButton(1) && (rightMotor.getEncVelocity() > MIN_ENC_VELOCITY && leftMotor.getEncVelocity() > MIN_ENC_VELOCITY))
					state = ShooterState.FIRING;
			}
			else
			{
				loaderY.set(false);
				rightMotor.set(0);
				leftMotor.set(0);
			}
		}
		else if (state == ShooterState.FIRING)
		{
			loaderX.set(true);
			rightMotor.set(0);
			leftMotor.set(0);
			state = ShooterState.FIRED;
		}
		else if (state == ShooterState.FIRED)
		{
			if (!j.getRawButton(1))
			{
				state = ShooterState.RELOADING;
			}
		}
		else if (state == ShooterState.RELOADING)
		{
			loaderY.set(false);
			loaderX.set(false);
			state = ShooterState.LOADED;
		}
		
		if(j.getRawButton(3))
			accumulator.set(-1);
		else
			accumulator.set(0);
	}
	
	public ShooterState getState()
	{
		return state;
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
