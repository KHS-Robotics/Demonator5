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
	
	private int error, prevError, accumulatedError;
	private double prevPidOut;
	
	private int autoSetpoint;
	private boolean isAtAutoSetpoint;
	
	public Shooter(Joystick j, Relay accumulator, CANTalon rightMotor, 
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
		autoMove(autoSetpoint);
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
					autoMove(holdSetpoint);
				} 
				else 
				{
					autoMove(setpoints.getSetpoint(buttonSelected));
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
	
	private void autoMove(int setpoint)
	{
		error = setpoint - enc.get();
		
		if (Math.abs(error) <= 20) 
		{
			isAtAutoSetpoint = true;
			return;
		}
		
		isAtAutoSetpoint = false;
		
		double out;
		
		if (error > 0)
			out = pid(ShooterPID.kP, ShooterPID.kI, ShooterPID.kD, error);
		else
			out = pid(ShooterPID.kPd, ShooterPID.kId, ShooterPID.kDd, error);
		
		arm.set(out);
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
	}
	
	private synchronized double pid(double p, double i, double d, int err) 
	{
		double out = 0;
		if (Math.abs(err) <= 5) 
		{
			accumulatedError = 0;
			return 0;
		} 
		else if (Math.abs(prevError - err) > Math.abs(err + prevError)) 
		{
			accumulatedError = 0;
		}

		accumulatedError += err;

		double P = p * err;
		double I = i * accumulatedError;
		double D = (err - prevError) * d;
		
		prevError = err;
		out = P + I + D;
		
		if (out > 1)
			out = 1;
		else if (out < -.5)
			out = -.5;
		
		if (out - prevPidOut > .1)
			out = prevPidOut + .1;
		else if (out - prevPidOut < -.1)
			out = prevPidOut - .1;
		
		
		prevPidOut = out;
		
		if(out < 0.1 && out > 0.0)
			out = 0.1;
		else if(out > -0.1 && out < 0.0)
			out = -0.1;
		
		return out;
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
