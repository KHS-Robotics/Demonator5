package org.usfirst.frc.team4342.api.shooter.controllers;

import org.usfirst.frc.team4342.api.shooter.ShooterState;

import edu.wpi.first.wpilibj.CANTalon;
import edu.wpi.first.wpilibj.Counter;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.Solenoid;

public class ShooterController 
{
	private Joystick switchBox;
	private CANTalon rightMotor, leftMotor;
	private Solenoid ballPusher;
	private Counter rightMotorCounter, leftMotorCounter;
	private DigitalInput ballSensor;
	private ArmController arm;
	
	private ShooterState state;
	
	private int numLoops;
	
	public ShooterController(Joystick switchBox, CANTalon rightMotor, CANTalon leftMotor, 
							Solenoid ballPusher, Counter rightMotorCounter, Counter leftMotorCounter, 
							DigitalInput ballSensor, ArmController arm)
	{
		this.rightMotor = rightMotor;
		this.leftMotor = leftMotor;
		this.ballPusher = ballPusher;
		this.rightMotorCounter = rightMotorCounter;
		this.leftMotorCounter = leftMotorCounter;
		this.ballSensor = ballSensor;
		this.arm = arm;
		
		state = ballPusher.get() ? ShooterState.FIRED : ShooterState.LOADED;
	}
	
	public void checkUser(int safetyButton, int fireButton)
	{
		if (state == ShooterState.LOADED)
		{
			if (switchBox.getRawButton(safetyButton))
			{
				rightMotor.set(1);
				leftMotor.set(1);
				
				arm.getAccumLifter().set(true);
				
				if (switchBox.getRawButton(fireButton))// && (rightMotorCounter.getPeriod() > MIN_ENC_VELOCITY && leftMotorCounter.getPeriod() > MIN_ENC_VELOCITY))
				{
					ballPusher.set(true);
					
					state = ShooterState.FIRING;
				}
			}
			else
			{
				rightMotor.set(0);
				leftMotor.set(0);
			}
		}
		else if (state == ShooterState.FIRING)
		{
			if(numLoops > 100)//!ballSensor.get())
			{
				rightMotor.set(0);
				leftMotor.set(0);
				
				arm.getAccumLifter().set(false);
				numLoops = 0;
				
				state = ShooterState.FIRED;
			}
			
			numLoops++;
		}
		else if (state == ShooterState.FIRED)
		{
			if (!switchBox.getRawButton(safetyButton))
			{
				rightMotor.set(-1.0);
				leftMotor.set(-1.0);
				
				arm.getAccumMotor().set(1.0);
				
				ballPusher.set(false);
				
				state = ShooterState.RELOADING;
			}
		}
		else if (state == ShooterState.RELOADING)
		{
			if(true)//ballSensor.get())
			{
				rightMotor.set(0.0);
				leftMotor.set(0.0);
				
				arm.getAccumMotor().set(0.0);
				
				state = ShooterState.LOADED;
			}
		}
	}
	
	public void setMotors(double output)
	{
		rightMotor.set(output);
		leftMotor.set(output);
	}
	
	public void setBallPusher(boolean on)
	{
		ballPusher.set(on);
	}
	
	public void stopAll()
	{
		rightMotor.set(0);
		leftMotor.set(0);
		ballPusher.set(false);
	}
	
	public ShooterState getState()
	{
		return state;
	}
}
