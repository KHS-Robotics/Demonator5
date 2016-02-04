package org.usfirst.frc.team4342.api.shooter;

import org.usfirst.frc.team4342.robot.components.Repository;

import edu.wpi.first.wpilibj.CANTalon;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.Relay;
import edu.wpi.first.wpilibj.Relay.Value;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.Ultrasonic;

public class Shooter 
{
	public static final int MIN_ENC_VELOCITY = 30;
	
	private Joystick j;
	private Relay accumulator;
	private CANTalon rightMotor, leftMotor, verticalMotor;
	private Solenoid loaderX, accumulatorLifter;
	private Ultrasonic ultra;
	
	private ShooterState state;
	
	public Shooter(Joystick j, Relay accumulator, CANTalon rightMotor, 
							   CANTalon leftMotor, CANTalon verticalMotor, 
							   Solenoid loaderX, Ultrasonic ultra)
	{
		this.j = j;
		this.accumulator = accumulator;
		this.rightMotor = rightMotor;
		this.leftMotor = leftMotor;
		this.verticalMotor = verticalMotor;
		this.loaderX = loaderX;
		this.ultra = ultra;
		
		state = loaderX.get() ? ShooterState.FIRED : ShooterState.LOADED;
	}
	
	public void handle()
	{
//		checkUserShooter();
//		checkUserAccumulator();
//		checkUserAngleMotor();
		
		basicFire();
		basicAccum();
	}
	
	private void checkUserShooter()
	{
		if (state == ShooterState.LOADED)
		{
			if (j.getRawButton(5))
			{
				rightMotor.set(1);
				leftMotor.set(1);
				
				accumulatorLifter.set(true);
				
				if (j.getRawButton(10) && (rightMotor.getEncVelocity() > MIN_ENC_VELOCITY && leftMotor.getEncVelocity() > MIN_ENC_VELOCITY))
					state = ShooterState.FIRING;
			}
			else
			{
				rightMotor.set(0);
				leftMotor.set(0);
			}
		}
		else if (state == ShooterState.FIRING)
		{
			loaderX.set(true);
			
			if(ultra.getRangeInches() > 18)
			{
				rightMotor.set(0);
				leftMotor.set(0);
				
				accumulatorLifter.set(false);
				
				state = ShooterState.FIRED;
			}
		}
		else if (state == ShooterState.FIRED)
		{
			if (!j.getRawButton(5))
			{
				state = ShooterState.RELOADING;
			}
		}
		else if (state == ShooterState.RELOADING)
		{
			loaderX.set(false);
			state = ShooterState.LOADED;
		}
	}
	
	private void checkUserAccumulator()
	{
		if(j.getRawButton(2))
		{
			accumulator.set(Value.kForward);
		}
		else
		{
			accumulator.set(Value.kOff);
		}
		
		if(j.getRawButton(3) || Repository.SwitchBox.getRawButton(2))
		{
			accumulatorLifter.set(true);
		}
		else
		{
			accumulatorLifter.set(false);
		}
	}
	
	private void checkUserAngleMotor()
	{
		verticalMotor.set(j.getY());
	}
	
	public void stopAll()
	{
		accumulator.set(Value.kOff);
		rightMotor.set(0);
		leftMotor.set(0);
		verticalMotor.set(0);
		loaderX.set(false);
		accumulatorLifter.set(false);
	}
	
	public ShooterState getState()
	{
		return state;
	}
	
	public Joystick getJoystick()
	{
		return j;
	}
	
	public Relay getAccumulator()
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
	
	public CANTalon getVerticalMotor()
	{
		return verticalMotor;
	}
	
	public Solenoid getLoaderX()
	{
		return loaderX;
	}
	
	public Solenoid getLoaderY()
	{
		return accumulatorLifter;
	}
	
	public void basicFire()
	{
		if (Repository.SwitchBox.getRawButton(3) && !Repository.SwitchBox.getRawButton(7))
		{
			rightMotor.set(0.75);
			leftMotor.set(0.75);
		}
		else if (!Repository.SwitchBox.getRawButton(3) && !Repository.SwitchBox.getRawButton(7))
		{
			rightMotor.set(0);
			leftMotor.set(0);
		}
	}
	
	public void basicAccum()
	{
		if (Repository.SwitchBox.getRawButton(7) && !(Repository.SwitchBox.getRawButton(3)))
		{
			accumulator.set(Value.kOn);
			rightMotor.set(-0.5);
			leftMotor.set(-0.5);
		}
		else
		{
			accumulator.set(Value.kOff);
		}
		
	}
}
