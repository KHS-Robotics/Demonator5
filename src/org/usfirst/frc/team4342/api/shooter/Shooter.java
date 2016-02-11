package org.usfirst.frc.team4342.api.shooter;

import org.usfirst.frc.team4342.robot.components.Repository;

import edu.wpi.first.wpilibj.CANTalon;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.Relay;
import edu.wpi.first.wpilibj.Relay.Value;
import edu.wpi.first.wpilibj.Solenoid;

public class Shooter 
{
	public static final int MIN_ENC_VELOCITY = 30;
	public static final double JOYSTICK_DEADBAND = 0.05;
	
	private Joystick j;
	private Relay accumulator;
	private CANTalon rightMotor, leftMotor, arm;
	private Solenoid ballPusher, accumulatorLifter;
	private Encoder enc;
	private DigitalInput ballSensor;
	private SetpointMapWrapper setpoints;
	
	private boolean hold, buttonPressed;
	private int buttonSelected, holdSetpoint;
	
	private ShooterState state;
	
	private int autoSetpoint;
	private boolean isAtAutoSetpoint, solenoidStatus;
	private double autoMotorOutput;
	
	private ArmPIDController apidc;
	
	public Shooter(Joystick j, CANTalon Accumulator, CANTalon rightMotor, 
					CANTalon leftMotor, CANTalon armMotor, Solenoid ballPusher,
					Encoder enc, DigitalInput ballSensor, SetpointMapWrapper setpoints)
	{
		this.j = j;
		this.accumulator = accumulator;
		this.rightMotor = rightMotor;
		this.leftMotor = leftMotor;
		this.arm = armMotor;
		this.ballPusher = ballPusher;
		this.enc = enc;
		this.ballSensor = ballSensor;
		this.setpoints = setpoints;
		
		apidc = new ArmPIDController(
			ArmPID.kP, 
			ArmPID.kI, 
			ArmPID.kD, 
			ArmPID.kPd, 
			ArmPID.kId, 
			ArmPID.kDd, 
			enc, 
			arm, 
			50.0
		);
		
		state = ballPusher.get() ? ShooterState.FIRED : ShooterState.LOADED;
	}
	
	public void handleTeleop()
	{
//		checkUserShooter();
//		checkUserAccumulator();
//		checkButtonStatus();
//		checkUserAngleMotor();
		
		basicFire();
		basicAccum();
	}
	
	public void handleAuto()
	{
		apidc.setSetpoint(autoSetpoint);
		setMotors(autoMotorOutput);
		setBallPusher(solenoidStatus);
	}
	
	public void stopAll()
	{
		accumulator.set(Value.kOff);
		rightMotor.set(0);
		leftMotor.set(0);
		arm.set(0);
		ballPusher.set(false);
		//accumulatorLifter.set(false);
	}
	
	public ShooterState getState()
	{
		return state;
	}
	
	public boolean isAtAutoSetpoint()
	{
		return isAtAutoSetpoint;
	}
	
	public Joystick getJoystick()
	{
		return j;
	}
	
	public Relay getAccumulator()
	{
		return accumulator;
	}
	
	public CANTalon getRightMotor()
	{
		return rightMotor;
	}
	
	public CANTalon getLeftMotor()
	{
		return leftMotor;
	}
	
	public CANTalon getArm()
	{
		return arm;
	}
	
	public Solenoid getBallPusher()
	{
		return ballPusher;
	}
	
	public Solenoid getAccumLifter()
	{
		return accumulatorLifter;
	}
	
	public void setAutoSetpoint(int setpoint)
	{
		this.autoSetpoint = setpoint;
	}
	
	public void setMotorOutput(double output)
	{
		this.autoMotorOutput = output;
	}
	
	public void setBallPusherValue(boolean on)
	{
		this.solenoidStatus = on;
	}
	
	private void setMotors(double output)
	{
		rightMotor.set(output);
		leftMotor.set(output);
	}
	
	private void setBallPusher(boolean on)
	{
		ballPusher.set(on);
	}
	
	private void checkUserShooter()
	{
		if (state == ShooterState.LOADED)
		{
			if (j.getRawButton(5))
			{
				rightMotor.set(1);
				leftMotor.set(1);
				
				accumulatorLifter.set(true);
				
				if (j.getRawButton(10))// && (rightMotor.getEncVelocity() > MIN_ENC_VELOCITY && leftMotor.getEncVelocity() > MIN_ENC_VELOCITY))
					state = ShooterState.FIRING;
			}
			else
			{
				rightMotor.set(0);
				leftMotor.set(0);
			}
		}
		else if (state == ShooterState.FIRING)
		{
			if(!ballSensor.get())
			{
				ballPusher.set(true);
				
				rightMotor.set(0);
				leftMotor.set(0);
				
				accumulatorLifter.set(false);
				
				state = ShooterState.FIRED;
			}
		}
		else if (state == ShooterState.FIRED)
		{
			if (!j.getRawButton(5))
			{
				state = ShooterState.RELOADING;
			}
		}
		else if (state == ShooterState.RELOADING)
		{
			if(ballSensor.get())
			{
				ballPusher.set(false);
				state = ShooterState.LOADED;
			}
		}
	}
	
	private void checkUserAccumulator()
	{
		if(j.getRawButton(2))
		{
			accumulator.set(Value.kForward);
		}
		else
		{
			accumulator.set(Value.kOff);
		}
		
		if(j.getRawButton(3) || Repository.SwitchBox.getRawButton(2))
		{
			accumulatorLifter.set(true);
		}
		else
		{
			accumulatorLifter.set(false);
		}
	}
	
	private void checkUserAngleMotor()
	{
		if(Math.abs(j.getY()) < JOYSTICK_DEADBAND) 
		{
			if(buttonPressed) 
			{
				if(hold) 
				{
					apidc.setSetpoint(holdSetpoint);
					apidc.enable();
				} 
				else 
				{
					apidc.setSetpoint(setpoints.getSetpoint(buttonSelected));
					apidc.enable();
				}
			} 
			else
			{
				arm.set(0.0);
			}
		} 
		else 
		{
			stopOperatorAutoMove();
			arm.set(j.getY());
		}
	}
	
	private void checkButtonStatus()
	{
		for(int i = 1; i < j.getButtonCount(); i++) 
		{
			if(j.getRawButton(i) && setpoints.containsButton(i)) 
			{
				hold = false;
				buttonPressed = true;
				buttonSelected = i;
			}
		}
		
		if(j.getRawButton(3)) 
		{
			buttonPressed = true;
			hold = true;
			buttonSelected = -1;
			holdSetpoint = enc.get();
		}
	}
	
	private void stopOperatorAutoMove() 
	{
		buttonPressed = false;
		buttonSelected = -1;
		apidc.disable();
	}
	
	public void basicFire()
	{
		if (Repository.SwitchBox.getRawButton(3) && !Repository.SwitchBox.getRawButton(7))
		{
			rightMotor.set(0.75);
			leftMotor.set(0.75);
		}
		else if (!Repository.SwitchBox.getRawButton(3) && !Repository.SwitchBox.getRawButton(7))
		{
			rightMotor.set(0);
			leftMotor.set(0);
		}
	}
	
	public void basicAccum()
	{
		if (Repository.SwitchBox.getRawButton(7) && !(Repository.SwitchBox.getRawButton(3)))
		{
			accumulator.set(Value.kOn);
			rightMotor.set(-0.5);
			leftMotor.set(-0.5);
		}
		else
		{
			accumulator.set(Value.kOff);
		}
	}
}
