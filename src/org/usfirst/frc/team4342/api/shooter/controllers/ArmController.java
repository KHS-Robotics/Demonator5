package org.usfirst.frc.team4342.api.shooter.controllers;

import org.usfirst.frc.team4342.api.shooter.arm.SetpointMapWrapper;
import org.usfirst.frc.team4342.api.shooter.arm.pid.ArmPID;
import org.usfirst.frc.team4342.api.shooter.arm.pid.ArmPIDController;

import edu.wpi.first.wpilibj.CANTalon;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.Solenoid;

public class ArmController 
{
	private static final double JOYSTICK_DEADBAND = 0.05;
	private static final double JOYSTICK_SENSITIVITY = 0.5;
	
	// TODO: get actual values; these are arbitrary
	private static final int TOP_WINDOW_SIZE = 140;
	private static final int BOTTOM_WINDOW_SIZE = 100;
	private static final int START_TOP_WINDOW = 100;
	private static final int START_BOTTOM_WINDOW = 400;
	
	private Joystick j, switchBox;
	private CANTalon armMotor, accumMotor;
	private Solenoid accumLifter;
	private Encoder enc;
	private DigitalInput topLS, botLS;
	private SetpointMapWrapper setpoints;
	
	private ArmPIDController apidc;
	
	private boolean buttonPressed, autoHold, goToSetpoint;
	private int buttonSelected, autoSetpoint;
	
	public ArmController(Joystick j, Joystick switchBox, CANTalon armMotor, CANTalon accumMotor, 
					Solenoid accumLifter, Encoder armEnc, DigitalInput topLS, DigitalInput botLS, SetpointMapWrapper setpoints)
	{
		this.j = j;
		this.switchBox = switchBox;
		this.armMotor = armMotor;
		this.accumMotor = accumMotor;
		this.accumLifter = accumLifter;
		this.enc = armEnc;
		this.topLS = topLS;
		this.botLS = botLS;
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
		if(topLS.get() && (j.getY() > 0 || armMotor.get() > 0))
		{
			//enc.reset();
			armMotor.set(0.0);
			return;
		}
		else if(botLS.get() && (j.getY() < 0 || armMotor.get() < 0))
		{
			armMotor.set(0.0);
			return;
		}
		
//		if(enc.get() < 140 && (j.getY() > 0 || armMotor.get() > 0))
//		{
//			armMotor.set(0.0);
//			disablePID();
//			return;
//		}
		
		
		if(!goToSetpoint)
		{
//			if(enc.get() < 50 && (armMotor.get() > 0 || j.getY() > 0))
//			{
//				disablePID();
//				armMotor.set(0);
//				return;
//			}
//			else if(enc.get() < 75 && (armMotor.get() > 0 || j.getY() > 0))
//			{
//				armMotor.set(0.1);
//			}
//			else if(enc.get() < 100 && (armMotor.get() > 0 || j.getY() > 0))
//			{
//				armMotor.set(0.2);
//				return;
//			}
//			else if(enc.get() > 450 && (armMotor.get() < 0 || j.getY() < 0))
//			{
//				disablePID();
//				armMotor.set(0.0);
//				return;
//			}
//			else if(enc.get() > 425 && (armMotor.get() < 0 || j.getY() < 0))
//			{
//				armMotor.set(-0.1);
//				return;
//			}
//			else if(enc.get() > 400 && (armMotor.get() < 0 || j.getY() < 0))
//			{
//				armMotor.set(-0.2);
//				return;
//			}
			
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
				
				armMotor.set(sensitivityControl(j.getY()));
				//armMotor.set(controlSpeed(sensitivityControl(j.getY()), enc.get()));
			}
		}
		else
		{
			apidc.setSetpoint(autoSetpoint);
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
	
	public void startAutomaticMode()
	{
		goToSetpoint = true;
		apidc.enable();
	}
	
	public void stopAutomaticMode()
	{
		goToSetpoint = false;
		apidc.disable();
	}
	
	public boolean isAtAutoSetpoint()
	{
		return apidc.onTarget();
	}
	
	public void setSetpoint(int setpoint)
	{
		this.autoSetpoint = setpoint;
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
	
	public void setPID(double p, double i, double d, double pd, double id, double dd)
	{
		apidc.setPIDUp(p, i, d);
		apidc.setPIDDown(pd, id, dd);
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
	
	/**
	 * Decelerates the arm speed as it approaches the top or
	 * bottom to prevent it from slamming harshly
	 * @param input the input from the joysticks or autoMove
	 * @param encCounts the current position of the elevator
	 * @return the new output for the motors
	 */
	private double controlSpeed(double input, int encCounts) 
	{
		double output = input;
		
		if(input < 0 && isInTopWindow(encCounts))
		{
			double penetration = (encCounts - START_TOP_WINDOW);
			output = input - (penetration*(input/(TOP_WINDOW_SIZE)));
			
			output = output > -.1 ? -.1 : output;
		}
		else if(input > 0 && isInBottomWindow(encCounts))
		{
			double penetration = (BOTTOM_WINDOW_SIZE - encCounts);
			output = input - (penetration*(input / (BOTTOM_WINDOW_SIZE)));
			
			output = output < .08 ? .08 : output;
		}
		
		return output;
	}
	
	/**
	 * Used to determine if the elevator is getting close to the bottom
	 * @param encCounts the current elevator position
	 * @return true if close, false otherwise
	 */
	private boolean isInBottomWindow(int encCounts) 
	{
		return encCounts <= START_BOTTOM_WINDOW;
	}
	
	/**
	 * Used to determine if the elevator is getting close to the top
	 * @param encCounts the current elevator postion
	 * @return true if close, false otherwise
	 */
	private boolean isInTopWindow(int encCounts) 
	{
		return encCounts >= START_TOP_WINDOW;
	}
	
	private void stopOperatorAutoMove() 
	{
		if(autoHold || buttonPressed)
		{
			apidc.disable();
		}
		
		autoHold = false;
		buttonPressed = false;
		buttonSelected = -1;
	}
	
	private double sensitivityControl(double input)
	{
		return (JOYSTICK_SENSITIVITY*Math.pow(input, 3))+((1-JOYSTICK_SENSITIVITY)*input);
	}
}
