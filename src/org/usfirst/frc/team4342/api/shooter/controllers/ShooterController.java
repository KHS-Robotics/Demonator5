package org.usfirst.frc.team4342.api.shooter.controllers;

import org.usfirst.frc.team4342.api.shooter.ShooterState;
import org.usfirst.frc.team4342.api.shooter.pid.ShooterPID;
import org.usfirst.frc.team4342.robot.components.Repository;

import edu.wpi.first.wpilibj.CANTalon;
import edu.wpi.first.wpilibj.Counter;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.PIDController;
import edu.wpi.first.wpilibj.PIDSourceType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class ShooterController 
{
	private Joystick switchBox;
	private CANTalon rightMotor, leftMotor;
	private Solenoid ballPusher;
	private Counter rightMotorCounter, leftMotorCounter;
	private DigitalInput ballSensor;
	private ArmController arm;
	
	private PIDController rightPID, leftPID;
	
	private ShooterState state;
	
	private int numLoops;
	
	public ShooterController(Joystick switchBox, CANTalon rightMotor, CANTalon leftMotor, 
							Solenoid ballPusher, Counter rightMotorCounter, Counter leftMotorCounter, 
							DigitalInput ballSensor, ArmController arm)
	{
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
		rightPID = new PIDController(0.0, ShooterPID.kI, ShooterPID.kD, ShooterPID.kF, this.rightMotorCounter, this.rightMotor);
		rightPID.setInputRange(0.0, 100.0);
		rightPID.setOutputRange(0.0, 1.0);
		
		leftMotor.setPIDSourceType(PIDSourceType.kRate);
		leftPID = new PIDController(0.0, ShooterPID.kI, ShooterPID.kD, ShooterPID.kF, this.leftMotorCounter, this.leftMotor);
		leftPID.setInputRange(0.0, 100.0);
		leftPID.setOutputRange(0.0, 1.0);
		
		enablePID();
		
		new ShooterFMonitor().start();
		
		state = ballPusher.get() ? ShooterState.FIRED : ShooterState.LOADED;
	}
	
	public void checkUser(int safetyButton, int fireButton)
	{
		if (state == ShooterState.LOADED)
		{
			if (switchBox.getRawButton(safetyButton))
			{
				leftPID.setSetpoint(SmartDashboard.getNumber("Shooter-Setpoint"));
				rightPID.setSetpoint(SmartDashboard.getNumber("Shooter-Setpoint"));
				
				arm.getAccumLifter().set(true);
				
				if (switchBox.getRawButton(fireButton) && (rightPID.onTarget() && leftPID.onTarget()))
				{
					ballPusher.set(true);
					
					state = ShooterState.FIRING;
				}
			}
			else
			{
				leftPID.setSetpoint(0);
				rightPID.setSetpoint(0);
			}
		}
		else if (state == ShooterState.FIRING)
		{
			if(numLoops > 50)//!ballSensor.get())
			{
				leftPID.setSetpoint(0);
				rightPID.setSetpoint(0);
				
				arm.getAccumLifter().set(false);
				numLoops = 0;
				
				disablePID();
				
				state = ShooterState.FIRED;
			}
			
			numLoops++;
		}
		else if (state == ShooterState.FIRED)
		{
			if (!switchBox.getRawButton(safetyButton))
			{
				rightMotor.set(-0.6);
				leftMotor.set(-0.6);
				
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
				
				enablePID();
				
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
	
	public void enablePID()
	{
		rightPID.enable();
		leftPID.disable();
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
	
	private class ShooterFMonitor extends Thread implements Runnable
	{
		@Override
		public void run()
		{
			while(true)
			{
				try
				{
					if(rightMotorCounter.getRate() > rightPID.getSetpoint())
					{
						rightPID.setPID(0.0, ShooterPID.kI, ShooterPID.kD, 0.0);
					}
					else
					{
						rightPID.setPID(0.0, ShooterPID.kI, ShooterPID.kD, ShooterPID.kF);
					}
					
					if(leftMotorCounter.getRate() > leftPID.getSetpoint())
					{
						leftPID.setPID(0.0, ShooterPID.kI, ShooterPID.kD, 0.0);
					}
					else
					{
						leftPID.setPID(0.0, ShooterPID.kI, ShooterPID.kD, ShooterPID.kF);
					}
					
					Thread.sleep(20);
				}
				catch(Exception ex)
				{
					Repository.Logs.error("Error while checking shooter setpoints", ex);
					break;
				}
			}
		}
	}
}
