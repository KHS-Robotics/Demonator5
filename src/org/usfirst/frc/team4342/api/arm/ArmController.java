package org.usfirst.frc.team4342.api.arm;

import org.usfirst.frc.team4342.api.arm.pid.ArmPID;
import org.usfirst.frc.team4342.api.arm.pid.ArmPIDController;
import org.usfirst.frc.team4342.api.arm.setpoints.SetpointMapWrapper;

import edu.wpi.first.wpilibj.CANTalon;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class ArmController 
{
	private static final double JOYSTICK_DEADBAND = 0.1;
	private static final double JOYSTICK_SENSITIVITY = 0.7;
	private static final double ARM_ARC_ANGLE = 140;
	private static final double DISTANCE_PER_PULSE = 1 / 944; // 1 / (7 distance per pulse * 142 gear ratio)
	
	private Joystick j, switchBox;
	private CANTalon armMotor, accumMotor;
	private Encoder enc;
	private DigitalInput topLS, botLS;
	private SetpointMapWrapper setpoints;

	private ArmPIDController apidc;

	private boolean buttonPressed, ranFirstAutoHold;
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
		
		apidc.setInputRange(-500, 0);
		apidc.setPercentTolerance(1);

		disablePID();
	}

	public void checkUser(int smartDashboardSetpointButton, int brakeButton, int accumInButton, int accumOutButton)
	{
		checkUserArm(smartDashboardSetpointButton, brakeButton);
		checkUserAccumulator(accumInButton, accumOutButton);
	}

	public void checkUserArm(int smartDashboardSetpointButton, int brakeButton)
	{
		double y = -j.getY();
		double encDist = Math.abs(enc.getDistance());
		double currentOutput = armMotor.get();
		
		if(topLS.get() && (y < 0 || currentOutput < 0))
		{
			disablePID();
			enc.reset();
			armMotor.set(0.0);
			return;
		}
		else if(botLS.get() && (y > 0 || currentOutput > 0))
		{
			disablePID();
			armMotor.set(0.0);
			return;
		}
		
		if(encDist < 25 && (y < 0 || currentOutput < 0))
		{
			disablePID();
			armMotor.set(-0.1);
			return;
		}
		else if(encDist < 190 && (y < 0 || currentOutput < 0))
		{
			disablePID();
			armMotor.set(-0.16);
			return;
		}
		else if(encDist > 420 && (y > 0 || currentOutput > 0))
		{
			disablePID();
			armMotor.set(0.16);
			return;
		}
		else if(encDist > 440 && (y > 0 || currentOutput > 0))
		{
			disablePID();
			armMotor.set(0.1);
			return;
		}
		
		if(!switchBox.getRawButton(smartDashboardSetpointButton))
		{
			if(Math.abs(y) < JOYSTICK_DEADBAND) 
			{
				checkButtonStatus();

				if(buttonPressed) 
				{
					setSetpoint(setpoints.getSetpoint(buttonSelected));
					enablePID();
				}
				else if(j.getRawButton(3))
				{
					setSetpoint(encDist - 10);
					enablePID();
				}
				else if(!switchBox.getRawButton(brakeButton))
				{
					if(!ranFirstAutoHold)
					{
						setSetpoint(encDist);
						enablePID();
						ranFirstAutoHold = true;
					}
				}
				else
				{
					disablePID();
					stopOperatorAutoMove();
					armMotor.set(0);
				}
			} 
			else 
			{
				stopOperatorAutoMove();
				
				if(y > 0 && encDist > 190)
				{
					armMotor.set(y / 5);
					return;
				}
				
				armMotor.set(y);
			}
		}
		else
		{
			stopOperatorAutoMove();
			enablePID();
			setSetpoint(SmartDashboard.getNumber("Arm-Setpoint"));
		}
	}

	public void checkUserAccumulator(int accumInButton, int accumOutButton)
	{
		if(j.getRawButton(accumInButton) || switchBox.getRawButton(accumInButton))
		{
			accumMotor.set(1);
		}
		else if(j.getRawButton(accumOutButton) || switchBox.getRawButton(accumOutButton))
		{
			accumMotor.set(-1);
		}
		else
		{
			accumMotor.set(0);
		}
	}
	
	public void setSetpoint(double setpoint)
	{
		apidc.setSetpoint(-setpoint);
	}

	public boolean isAtAutoSetpoint()
	{
		return Math.abs(apidc.getError()) <= 5;
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
		double relativeArmAngle = ARM_ARC_ANGLE - angle;
		double raw = relativeArmAngle / (90*DISTANCE_PER_PULSE);
		double distance = raw * .25 * DISTANCE_PER_PULSE;
		
		setSetpoint(distance);
	}

	public double getAngle()
	{
		return (enc.getRaw() * 0.25 * DISTANCE_PER_PULSE) * 360;
	}

	private void checkButtonStatus()
	{
		for(int i = 1; i < j.getButtonCount(); i++) 
		{
			if(j.getRawButton(i) && setpoints.containsButton(i)) 
			{
				ranFirstAutoHold = false;
				buttonPressed = true;
				buttonSelected = i;
			}
		}
	}

	public double getPortCullisVelocity()
	{
		return -enc.getRate()*Math.cos(enc.getDistance()*Math.PI)*18;
	}

	private void stopOperatorAutoMove() 
	{
		if(ranFirstAutoHold || buttonPressed)
		{
			disablePID();
		}

		ranFirstAutoHold = false;
		buttonPressed = false;
		buttonSelected = -1;
	}
	
	public void stopAll()
	{
		stopOperatorAutoMove();
		armMotor.set(0);
		accumMotor.set(0);
	}
	
	private double sensitivityControl(double input)
	{
		return (JOYSTICK_SENSITIVITY*Math.pow(input, 3))+((1-JOYSTICK_SENSITIVITY)*input);
	}
}
