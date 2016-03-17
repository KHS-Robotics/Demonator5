package org.usfirst.frc.team4342.api.shooter;

import org.usfirst.frc.team4342.api.shooter.pid.ShooterPID;
import org.usfirst.frc.team4342.robot.components.Repository;
import org.usfirst.frc.team4342.api.arm.ArmController;

import edu.wpi.first.wpilibj.CANTalon;
import edu.wpi.first.wpilibj.Counter;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.PIDController;
import edu.wpi.first.wpilibj.PIDSourceType;
import edu.wpi.first.wpilibj.Solenoid;

public class ShooterController 
{
	public enum ShooterState
	{
		LOADED, FIRING
	}
	
	private Joystick driveStick, switchBox;
	private CANTalon rightMotor, leftMotor;
	private Solenoid ballPusher;
	private Counter rightMotorCounter, leftMotorCounter;
	private ArmController arm;
	
	private PIDController rightPID, leftPID;
	
	private ShooterState state = ShooterState.LOADED;
	
	private int numLoops, driveShootLoops;
	private boolean driverShooting, userFired;
	
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
	}
	
	public void checkUser(int driverShootButton, int safetyButton, int fireButton, int accumInButton, int accumOutButton)
	{
		if(driverShooting || (driveStick.getRawButton(driverShootButton) && state == ShooterState.LOADED))
		{
			driverShooting = true;
			
			enablePID();
			
			leftPID.setSetpoint(75);
			rightPID.setSetpoint(75);
			
			driveShootLoops++;
			
			if(isAtSetpoint())
			{
				setBallPusher(true);
				
				if(driveShootLoops > 20)
				{
					setMotorsPID(0);
					setBallPusher(false);
					
					driveShootLoops = 0;
					driverShooting = false;
				}
			}
		}
		else if(!driverShooting)
		{
			if (state == ShooterState.LOADED)
			{
				if (switchBox.getRawButton(safetyButton) && !userFired)
				{
					enablePID();
					leftPID.setSetpoint(85);
					rightPID.setSetpoint(85);
					
					if (switchBox.getRawButton(fireButton) && isAtSetpoint())
					{
						setBallPusher(true);
						state = ShooterState.FIRING;
					}
				}
				else if(switchBox.getRawButton(accumInButton) && (!Repository.DriveStick.getRawButton(accumOutButton) || !switchBox.getRawButton(accumOutButton)))
				{
					setMotors(-0.67);
					setAccumulatorMotor(1);
				}
				else
				{
					setBallPusher(false);
					stopAllMotors();
					userFired = false;
				}
			}
			else if (state == ShooterState.FIRING)
			{
				if(numLoops > 10)
				{
					setMotorsPID(0);
					numLoops = 0;
					setBallPusher(false);
					
					userFired = true;
					
					state = ShooterState.LOADED;
				}
				
				numLoops++;
			}
		}
	}
	
	public void fire()
	{
		if(state == ShooterState.LOADED && !driverShooting)
		{
			enablePID();
			setMotorsPID(85);
			
			while(!isAtSetpoint()) {
				// Chillin' like a villain
			}
			
			setBallPusher(true);
		}
	}
	
	public void setMotors(double output)
	{
		rightMotor.set(output);
		leftMotor.set(output);
	}
	
	public void setMotorsPID(double setpoint)
	{
		rightMotor.set(setpoint);
		leftMotor.set(setpoint);
	}
	
	public void setAccumulatorMotor(double output)
	{
		arm.getAccumMotor().set(output);
	}
	
	public void setBallPusher(boolean on)
	{
		if(on && (rightMotor.get() <= 0 || leftMotor.get() <= 0))
			return;
		
		ballPusher.set(on);
	}
	
	public void enablePID()
	{
		rightPID.enable();
		leftPID.enable();
	}
	
	public void disablePID()
	{
		if(rightPID.isEnabled())
			rightPID.disable();
		
		if(leftPID.isEnabled())
			leftPID.disable();
	}
	
	public void setPID(double p, double i, double d)
	{
		rightPID.setPID(p, i, d);
		leftPID.setPID(p, i, d);
	}
	
	public boolean isAtSetpoint()
	{
		boolean right = Math.abs(rightPID.getError()) < 2;
		boolean left = Math.abs(leftPID.getError()) < 2;
		
		return right && left;
	}
	
	public void stopAllMotors()
	{
		if(rightPID.isEnabled() || leftPID.isEnabled())
			disablePID();
		
		rightMotor.set(0);
		leftMotor.set(0);
		arm.getAccumMotor().set(0);
	}
	
	public ShooterState getState()
	{
		return state;
	}
}
