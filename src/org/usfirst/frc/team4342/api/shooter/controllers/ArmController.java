package org.usfirst.frc.team4342.api.shooter.controllers;

import org.usfirst.frc.team4342.api.shooter.arm.SetpointMapWrapper;
import org.usfirst.frc.team4342.api.shooter.arm.pid.ArmPID;
import org.usfirst.frc.team4342.api.shooter.arm.pid.ArmPIDController;
import org.usfirst.frc.team4342.robot.components.Repository;

import edu.wpi.first.wpilibj.CANTalon;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class ArmController 
{
	private static final double JOYSTICK_DEADBAND = 0.05;
	
	private Joystick j, switchBox;
	private CANTalon armMotor, accumMotor;
	private Solenoid accumLifter;
	private Encoder enc;
	private SetpointMapWrapper setpoints;
	
	private ArmPIDController apidc;
	
	private boolean buttonPressed, autoHold;
	private int buttonSelected;
	
	public ArmController(Joystick j, Joystick switchBox, CANTalon armMotor, CANTalon accumMotor, 
					Solenoid accumLifter, Encoder armEnc, SetpointMapWrapper setpoints)
	{
		this.j = j;
		this.switchBox = switchBox;
		this.armMotor = armMotor;
		this.accumMotor = accumMotor;
		this.accumLifter = accumLifter;
		this.enc = armEnc;
		this.setpoints = setpoints;
		
		apidc = new ArmPIDController(
			ArmPID.kP, 
			ArmPID.kI, 
			ArmPID.kD, 
			ArmPID.kPd, 
			ArmPID.kId, 
			ArmPID.kDd, 
			armEnc, 
			armMotor,
			0.05
		);
		
		apidc.setPercentTolerance(5);
		
		apidc.disable();
	}
	
	public void checkUser(int brakeButton, int accumButton, int accumLiftButton, int safetyButton)
	{
		checkUserArm(brakeButton);
		checkUserAccumulator(accumButton, accumLiftButton, safetyButton);
	}
	
	public void checkUserArm(int brakeButton)
	{
		if(Math.abs(j.getY()) < JOYSTICK_DEADBAND) 
		{
			checkButtonStatus();
			
			if(buttonPressed) 
			{
				apidc.setSetpoint(setpoints.getSetpoint(buttonSelected));
 				apidc.enable();
 			}
 			else if(!autoHold)
  			{
 				apidc.setSetpoint(enc.get());
 				apidc.enable();
 				autoHold = true;
  			}
		} 
		else 
		{
			stopOperatorAutoMove();
			armMotor.set(j.getY());
		}
	}
	
	public void checkUserAccumulator(int accumButton, int accumLiftButton, int fireSafetyButton)
	{
		if(j.getRawButton(accumButton) || switchBox.getRawButton(accumButton))
		{
			accumMotor.set(1);
		}
		else
		{
			accumMotor.set(0);
		}
		
		if(j.getRawButton(accumLiftButton) || switchBox.getRawButton(accumLiftButton))
		{
			accumLifter.set(true);
		}
		else if (!switchBox.getRawButton(fireSafetyButton))
		{
			accumLifter.set(false);
		}
	}
	
	public boolean isAtAutoSetpoint()
	{
		return apidc.onTarget();
	}
	
	public void setSetpoint(double setpoint)
	{
		apidc.setSetpoint(setpoint);
	}
	
	public void setAccumLifter(boolean on)
	{
		accumLifter.set(on);
	}
	
	public void stopAll()
	{
		stopOperatorAutoMove();
		armMotor.set(0);
		accumMotor.set(0);
		accumLifter.set(false);
	}
	
	public void enablePID()
	{
		apidc.enable();
	}
	
	public void disablePID()
	{
		apidc.disable();
	}
	
	public CANTalon getAccumMotor()
	{
		return accumMotor;
	}
	
	public Solenoid getAccumLifter()
	{
		return accumLifter;
	}
	
	public ArmPIDController getPIDController()
	{
		return apidc;
	}
	
	private void checkButtonStatus()
	{
		for(int i = 1; i < j.getButtonCount(); i++) 
		{
			if(j.getRawButton(i) && setpoints.containsButton(i)) 
			{
				autoHold = false;
				buttonPressed = true;
				buttonSelected = i;
			}
		}
	}
	
	private void stopOperatorAutoMove() 
	{
		autoHold = false;
		buttonPressed = false;
		buttonSelected = -1;
		apidc.disable();
	}
}
