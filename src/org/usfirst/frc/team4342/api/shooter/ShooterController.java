package org.usfirst.frc.team4342.api.shooter;

import org.usfirst.frc.team4342.api.shooter.arm.ArmController;
import org.usfirst.frc.team4342.robot.Repository;

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
	
	private Joystick switchBox;
	private CANTalon rightMotor, leftMotor;
	private Solenoid ballPusher;
	private Counter rightMotorCounter, leftMotorCounter;
	private ArmController arm;
	
	private PIDController rightPID, leftPID;
	
	private ShooterState state = ShooterState.LOADED;
	
	private int numLoops;
	private boolean userFired;
	
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
	
	public void checkUser(int highGoalSafety, int lowGoalSafety, int fireButton, int accumInButton, int accumOutButton)
	{
		if (state == ShooterState.LOADED)
		{
			if(switchBox.getRawButton(highGoalSafety) && !userFired)
			{
				enablePID();
				setSetpoint(85);
				
				if (switchBox.getRawButton(fireButton) && isAtSetpoint())
				{
					setBallPusher(true);
					state = ShooterState.FIRING;
				}
			}
			else if(switchBox.getRawButton(lowGoalSafety) && !userFired)
			{
				enablePID();
				setSetpoint(60);
				
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
				setSetpoint(0);
				numLoops = 0;
				setBallPusher(false);
				
				userFired = true;
				
				state = ShooterState.LOADED;
			}
			
			numLoops++;
		}
	}
	
	public void setMotors(double output)
	{
		rightMotor.set(output);
		leftMotor.set(output);
	}
	
	public void setSetpoint(double setpoint)
	{
		rightPID.setSetpoint(setpoint);
		leftPID.setSetpoint(setpoint);
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
