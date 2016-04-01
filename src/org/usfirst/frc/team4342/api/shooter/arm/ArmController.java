package org.usfirst.frc.team4342.api.shooter.arm;

import edu.wpi.first.wpilibj.CANTalon;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class ArmController 
{
	private static final double JOYSTICK_DEADBAND = 0.05;
	private static final double ARM_ARC_ANGLE = 140;
	private static final double DISTANCE_PER_PULSE = 1 / 944; // 1 / (7 distance per pulse * 142 gear ratio)
	
	private Joystick j, switchBox;
	private CANTalon armMotor, accumMotor;
	private Encoder enc;
	private DigitalInput topLS, botLS;

	private ArmPIDController apidc;

	private boolean buttonPressed, ranFirstAutoHold;
	private int selectedSetpoint;
	private double currentHoldSetpoint;

	public ArmController(Joystick j, Joystick switchBox, CANTalon armMotor, CANTalon accumMotor, 
			Encoder armEnc, DigitalInput topLS, DigitalInput botLS)
	{
		this.j = j;
		this.switchBox = switchBox;
		this.armMotor = armMotor;
		this.accumMotor = accumMotor;
		this.enc = armEnc;
		this.topLS = topLS;
		this.botLS = botLS;
		
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
		
		apidc.setInputRange(-480, 0);
		apidc.setPercentTolerance(1);

		disablePID();
	}

	public void checkUser(int smartDashboardSetpointButton, int accumInButton, int accumOutButton)
	{
		checkUserArm(smartDashboardSetpointButton);
		checkUserAccumulator(accumInButton, accumOutButton);
	}

	public void checkUserArm(int smartDashboardSetpointButton)
	{
		if(!switchBox.getRawButton(smartDashboardSetpointButton))
		{
			checkButtonStatus();

			if(buttonPressed) 
			{
				setSetpoint(selectedSetpoint);
				enablePID();
			}
			
			double y = -j.getY();
			double encDist = Math.abs(enc.getDistance());
			
			if(topLS.get() && y < 0)
			{
				disablePID();
				enc.reset();
				armMotor.set(0.0);
				return;
			}
			else if(botLS.get() && y > 0)
			{
				disablePID();
				armMotor.set(0.0);
				return;
			}
			
			if(Math.abs(y) < JOYSTICK_DEADBAND && !buttonPressed) 
			{
				if(encDist > 190 && encDist < 455)
				{
					if(!ranFirstAutoHold)
					{
						setSetpoint(encDist);
						enablePID();
						ranFirstAutoHold = true;
					}
				}
				else if(encDist < 25)
				{
					disablePID();
					armMotor.set(-0.12);
					return;
				}
				else if(encDist <= 190)
				{
					disablePID();
					armMotor.set(-0.2);
					return;
				}
				else if(encDist > 450)
				{
					disablePID();
					armMotor.set(0.2);
					return;
				}
				else if(encDist > 460)
				{
					disablePID();
					armMotor.set(0.12);
					return;
				}
			} 
			else if(Math.abs(y) >= JOYSTICK_DEADBAND)
			{
				stopOperatorAutoMove();
				
				if(encDist < 25 && y < 0)
				{
					disablePID();
					armMotor.set(-0.1);
					return;
				}
				else if(encDist <= 190 && y < 0)
				{
					disablePID();
					armMotor.set(-0.16);
					return;
				}
				else if(encDist > 450 && y >= 0)
				{
					disablePID();
					armMotor.set(0.16);
					return;
				}
				else if(encDist > 460 && y >= 0)
				{
					disablePID();
					armMotor.set(0.1);
					return;
				}
				
				if(y > 0 && encDist >= 190)
				{
					armMotor.set(y / 5);
					return;
				}
				
				armMotor.set(y);
			}
		}
		else
		{
			ranFirstAutoHold = false;
			buttonPressed = false;
			enablePID();
			setSetpoint(SmartDashboard.getNumber("Arm-Setpoint"));
		}
	}

	public void checkUserAccumulator(int accumInButton, int accumOutButton)
	{
		if(j.getRawButton(accumOutButton) || switchBox.getRawButton(accumOutButton))
		{
			accumMotor.set(-1);
		}
		else if(!switchBox.getRawButton(accumInButton))
		{
			accumMotor.set(0);
		}
	}
	
	public void setSetpoint(double setpoint)
	{
		currentHoldSetpoint = setpoint;
		apidc.setSetpoint(-setpoint);
	}

	public boolean isAtAutoSetpoint()
	{
		return Math.abs(apidc.getError()) <= 10;
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
		if(j.getRawButton(4))
		{
			buttonPressed = true;
			selectedSetpoint = 210;
		}
		else if(j.getRawButton(6))
		{
			buttonPressed = true;
			selectedSetpoint = 395;
		}
		else if(j.getRawButton(3))
		{
			buttonPressed = true;
			selectedSetpoint = 0;
		}
		else if(j.getRawButton(5))
		{
			buttonPressed = true;
			selectedSetpoint = 470;
		}
		else if(ranFirstAutoHold && j.getRawButton(2)) 
		{
			setSetpoint(currentHoldSetpoint+10);
		}
		else if(ranFirstAutoHold && j.getRawButton(7))
		{
			setSetpoint(currentHoldSetpoint-10);
		}
	}

	public double getPortCullisVelocity()
	{
		return -enc.getRate()*Math.cos(enc.getDistance()*Math.PI)*18;
	}

	private void stopOperatorAutoMove() 
	{
		disablePID();
		ranFirstAutoHold = false;
		buttonPressed = false;
	}
	
	public void stopAll()
	{
		stopOperatorAutoMove();
		armMotor.set(0);
		accumMotor.set(0);
	}
}
