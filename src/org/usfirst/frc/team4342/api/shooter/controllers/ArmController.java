package org.usfirst.frc.team4342.api.shooter.controllers;

import org.usfirst.frc.team4342.api.shooter.arm.SetpointMapWrapper;
import org.usfirst.frc.team4342.api.shooter.arm.pid.ArmPID;
import org.usfirst.frc.team4342.api.shooter.arm.pid.ArmPIDController;

import edu.wpi.first.wpilibj.CANTalon;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class ArmController 
{
	private static final double JOYSTICK_DEADBAND = 0.05;
	private static final double JOYSTICK_SENSITIVITY = 0.2;

	private static final int TOP_WINDOW_SIZE = 180;
	private static final int BOTTOM_WINDOW_SIZE = 140;
	private static final int START_TOP_WINDOW = 300;
	private static final int START_BOTTOM_WINDOW = 140;

	private Joystick j, switchBox;
	private CANTalon armMotor, accumMotor;
	private Encoder enc;
	private DigitalInput topLS, botLS;
	private SetpointMapWrapper setpoints;

	private ArmPIDController apidc;

	private boolean buttonPressed, autoHold;
	private int buttonSelected;

	public ArmController(Joystick j, Joystick switchBox, CANTalon armMotor, CANTalon accumMotor, 
			Encoder armEnc, DigitalInput topLS, DigitalInput botLS, SetpointMapWrapper setpoints)
	{
		this.j = j;
		this.switchBox = switchBox;
		this.armMotor = armMotor;
		this.accumMotor = accumMotor;
		this.enc = armEnc;
		this.topLS = topLS;
		this.botLS = botLS;
		this.setpoints = setpoints;
		
		SmartDashboard.putNumber("Arm-Setpoint", 0.0);

		apidc = new ArmPIDController(
			ArmPID.kP, 
			ArmPID.kI, 
			ArmPID.kD, 
			ArmPID.kPd, 
			ArmPID.kId, 
			ArmPID.kDd,
			armEnc,
			armEnc, 
			armMotor,
			0.05
		);
		
		apidc.setInputRange(0, 500);
		apidc.setPercentTolerance(5);

		disablePID();
	}

	public void checkUser(int smartDashboardSetpointButton, int accumButton, int accumLiftButton, int safetyButton)
	{
		checkUserArm(smartDashboardSetpointButton);
		checkUserAccumulator(accumButton, accumLiftButton, safetyButton);
	}

	public void checkUserArm(int smartDashboardSetpointButton)
	{
		if(!switchBox.getRawButton(smartDashboardSetpointButton))
		{
			if(enc.get() < 140 && (j.getY() > 0 || armMotor.get() > 0))
			{
				armMotor.set(0);
				disablePID();
				return;
			}
			else if(enc.get() > 375 && (j.getY() < 0 || armMotor.get() < 0))
			{
				armMotor.set(0);
				disablePID();
				return;
			}

			if(topLS.get() && (j.getY() > 0 || armMotor.get() > 0))
			{
				enc.reset();
				armMotor.set(0.0);
				return;
			}
			else if(botLS.get() && (j.getY() < 0 || armMotor.get() < 0))
			{
				armMotor.set(0.0);
				return;
			}

			if(Math.abs(j.getY()) < JOYSTICK_DEADBAND) 
			{
				checkButtonStatus();

				if(buttonPressed) 
				{
					setSetpoint(setpoints.getSetpoint(buttonSelected));
					apidc.enable();
				}
				else if(enc.get() > 140)
				{
					if(!autoHold)
					{
						setSetpoint(enc.get());
						apidc.enable();
						autoHold = true;
					}

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
			setSetpoint(SmartDashboard.getNumber("Arm-Setpoint"));
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
	}

	public boolean isAtAutoSetpoint()
	{
		return (enc.get() > apidc.getSetpoint()-3.0) || (enc.get() < apidc.getSetpoint()+3.0);
	}

	public void setSetpoint(double setpoint)
	{
		apidc.setSetpoint(setpoint);
	}

	public void stopAll()
	{
		stopOperatorAutoMove();
		armMotor.set(0);
		accumMotor.set(0);
	}

	public void enablePID()
	{
		apidc.enable();
	}

	public void disablePID()
	{
		if(apidc.isEnabled())
			apidc.disable();
	}

	public CANTalon getAccumMotor()
	{
		return accumMotor;
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

	public void setAngle(double angle)
	{
		setSetpoint(angle / 360.0);
	}

	public double getAngle()
	{
		return enc.getDistance() * 360;
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

	public double getPortCullisVelocity()
	{
		return -enc.getRate()*Math.cos(enc.getDistance()*Math.PI)*18;
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
			return 0.0;
		}

		return output;
	}

	/**
	 * Used to determine if the arm is getting close to the bottom
	 * @param encCounts the current arm position
	 * @return true if within the window, false otherwise
	 */
	private boolean isInBottomWindow(int encCounts) 
	{
		return encCounts <= START_BOTTOM_WINDOW;
	}

	/**
	 * Used to determine if the arm is getting close to the top
	 * @param encCounts the current arm position
	 * @return true if within the window, false otherwise
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
