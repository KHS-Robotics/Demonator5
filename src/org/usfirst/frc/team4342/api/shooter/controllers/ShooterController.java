package org.usfirst.frc.team4342.api.shooter.controllers;

import org.usfirst.frc.team4342.api.shooter.ShooterState;
import org.usfirst.frc.team4342.api.shooter.pid.ShooterPID;

import edu.wpi.first.wpilibj.CANTalon;
import edu.wpi.first.wpilibj.Counter;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.PIDController;
import edu.wpi.first.wpilibj.PIDSourceType;
import edu.wpi.first.wpilibj.Solenoid;

public class ShooterController 
{
	private Joystick driveStick, switchBox;
	private CANTalon rightMotor, leftMotor;
	private Solenoid ballPusher;
	private Counter rightMotorCounter, leftMotorCounter;
	private DigitalInput ballSensor;
	private ArmController arm;
	
	private PIDController rightPID, leftPID;
	
	private ShooterState state;
	
	private int numLoops, driveShootLoops;
	private boolean driverShooting;
	
	public ShooterController(Joystick driveStick, Joystick switchBox, CANTalon rightMotor, CANTalon leftMotor, 
							Solenoid ballPusher, Counter rightMotorCounter, Counter leftMotorCounter, 
							DigitalInput ballSensor, ArmController arm)
	{
		this.driveStick = driveStick;
		this.switchBox = switchBox;
		this.rightMotor = rightMotor;
		this.leftMotor = leftMotor;
		this.ballPusher = ballPusher;
		this.rightMotorCounter = rightMotorCounter;
		this.leftMotorCounter = leftMotorCounter;
		this.ballSensor = ballSensor;
		this.arm = arm;
		
		rightMotorCounter.setPIDSourceType(PIDSourceType.kRate);
		leftMotorCounter.setPIDSourceType(PIDSourceType.kRate);
		
		rightMotor.setPIDSourceType(PIDSourceType.kRate);
		rightPID = new PIDController(ShooterPID.kP, ShooterPID.kI, ShooterPID.kD, ShooterPID.kF, this.rightMotorCounter, this.rightMotor);
		rightPID.setPercentTolerance(2);
		rightPID.setInputRange(0.0, 100.0);
		rightPID.setOutputRange(0.0, 1.0);
		
		leftMotor.setPIDSourceType(PIDSourceType.kRate);
		leftPID = new PIDController(ShooterPID.kP, ShooterPID.kI, ShooterPID.kD, ShooterPID.kF, this.leftMotorCounter, this.leftMotor);
		leftPID.setPercentTolerance(2);
		leftPID.setInputRange(0.0, 100.0);
		leftPID.setOutputRange(0.0, 1.0);
		
		enablePID();
		
		state = ballPusher.get() ? ShooterState.RELOADING : ShooterState.LOADED;
	}
	
	public void checkUser(int driverShootButton, int safetyButton, int fireButton, int accumButton)
	{
		if(driverShooting || (driveStick.getRawButton(driverShootButton) && state == ShooterState.LOADED))
		{
			driverShooting = true;
			
			enablePID();
			
			// set these to 100 to get them spinning faster (more error)
			// Don't worry it's only for a split second
			leftPID.setSetpoint(100);
			rightPID.setSetpoint(100);
			
			driveShootLoops++;
			
			if(driveShootLoops > 20)
			{
				ballPusher.set(true);
				
				if(driveShootLoops > 30)
				{
					ballPusher.set(false);
					leftPID.setSetpoint(0);
					rightPID.setSetpoint(0);
					disablePID();
					
					driveShootLoops = 0;
					driverShooting = false;
				}
			}
		}
		else if(!driverShooting)
		{
			if (state == ShooterState.LOADED)
			{
				if (switchBox.getRawButton(safetyButton))
				{
					enablePID();
					leftPID.setSetpoint(85);
					rightPID.setSetpoint(85);
					
					if (switchBox.getRawButton(fireButton) && isAtSetpoint())
					{
						ballPusher.set(true);
						
						state = ShooterState.FIRING;
					}
				}
				else if(switchBox.getRawButton(accumButton))
				{
					rightMotor.set(-0.5);
					leftMotor.set(-0.5);
					arm.getAccumMotor().set(1.0);
				}
				else
				{
					leftPID.setSetpoint(0);
					rightPID.setSetpoint(0);
					disablePID();
					rightMotor.set(0);
					leftMotor.set(0);
					arm.getAccumMotor().set(0);
				}
			}
			else if (state == ShooterState.FIRING)
			{
				if(numLoops > 50)//!ballSensor.get())
				{
					leftPID.setSetpoint(0);
					rightPID.setSetpoint(0);
					rightMotor.set(0);
					leftMotor.set(0);
					
					numLoops = 0;
					
					disablePID();
					
					state = ShooterState.RELOADING;
				}
				
				numLoops++;
			}
			else if (state == ShooterState.RELOADING)
			{
				if(true)//ballSensor.get())
				{
					ballPusher.set(false);
					
					enablePID();
					
					state = ShooterState.LOADED;
				}
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
	
	public void enablePID()
	{
		rightPID.enable();
		leftPID.enable();
	}
	
	public void disablePID()
	{
		rightPID.disable();
		leftPID.disable();
	}
	
	public void setPID(double p, double i, double d)
	{
		rightPID.setPID(p, i, d);
		leftPID.setPID(p, i, d);
	}
	
	public boolean isAtSetpoint()
	{
		boolean right = (rightMotorCounter.getRate() > rightPID.getSetpoint() - 2.0 || rightMotorCounter.getRate() < rightPID.getSetpoint() + 2.0);
		boolean left = (leftMotorCounter.getRate() > leftPID.getSetpoint() - 2.0 || leftMotorCounter.getRate() < leftPID.getSetpoint() + 2.0);
		
		return right && left;
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
