package org.usfirst.frc.team4342.api.shooter;

import org.usfirst.frc.team4342.robot.components.Repository;

import edu.wpi.first.wpilibj.CANTalon;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.Solenoid;

public class Shooter 
{
	public static final int MIN_ENC_VELOCITY = 30;
	
	private Joystick j;
	private CANTalon accumulator, rightMotor, leftMotor, verticalMotor;
	private Solenoid loaderX, loaderY;
	
	private boolean sendShootCANMssg, sendShootCancelCANMssg;
	private boolean sendAccumCANMssg, sendAccumStopMssg;
	private boolean sendLoaderYMssg, sendLoaderYStopMssg;
	
	private ShooterState state;
	
	public Shooter(Joystick j, CANTalon accumulator, CANTalon rightMotor, 
							   CANTalon leftMotor, Solenoid loaderX,
							   CANTalon verticalMotor)
	{
		this.j = j;
		this.accumulator = accumulator;
		this.rightMotor = rightMotor;
		this.leftMotor = leftMotor;
		this.verticalMotor = verticalMotor;
		this.loaderX = loaderX;
		
		state = loaderX.get() ? ShooterState.FIRED : ShooterState.LOADED;
	}
	
	public void handle()
	{
//		checkUserShooter();
//		checkUserAccumulator();
//		checkUserAngleMotor();
		
		basicFire();
		basicAccum();
	}
	
	private void checkUserShooter()
	{
		if (state == ShooterState.LOADED)
		{
			if (sendShootCANMssg && j.getRawButton(5))
			{
				rightMotor.set(1);
				leftMotor.set(1);
				
				if (j.getRawButton(10) && (rightMotor.getEncVelocity() > MIN_ENC_VELOCITY && leftMotor.getEncVelocity() > MIN_ENC_VELOCITY)) {
					state = ShooterState.FIRING;
				}
				
				sendShootCANMssg = false;
				sendShootCancelCANMssg = true;
			}
			else if(sendShootCancelCANMssg)
			{
				rightMotor.set(0);
				leftMotor.set(0);
				
				sendShootCANMssg = true;
				sendShootCancelCANMssg = false;
			}
		}
		else if (state == ShooterState.FIRING)
		{
			loaderX.set(true);
			rightMotor.set(0);
			leftMotor.set(0);
			state = ShooterState.FIRED;
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
			loaderX.set(false);
			state = ShooterState.LOADED;
		}
	}
	
	private void checkUserAccumulator()
	{
		if(sendAccumCANMssg && j.getRawButton(2))
		{
			accumulator.set(-1);
			sendAccumStopMssg = true;
			sendAccumCANMssg = false;
		}
		else if(sendAccumStopMssg)
		{
			accumulator.set(0);
			sendAccumCANMssg = true;
			sendAccumStopMssg = false;
		}
		
		if(sendLoaderYMssg && (j.getRawButton(3) || Repository.SwitchBox.getRawButton(2)))
		{
			loaderY.set(true);
			sendLoaderYMssg = false;
			sendLoaderYStopMssg = true;
		}
		else if(sendLoaderYStopMssg)
		{
			loaderY.set(false);
			sendLoaderYMssg = true;
			sendLoaderYStopMssg = false;
		}
	}
	
	private void checkUserAngleMotor()
	{
		verticalMotor.set(j.getY());
	}
	
	public void stopAll()
	{
		accumulator.set(0);
		rightMotor.set(0);
		leftMotor.set(0);
		verticalMotor.set(0);
		loaderX.set(false);
		loaderY.set(false);
	}
	
	public ShooterState getState()
	{
		return state;
	}
	
	public Joystick getJoystick()
	{
		return j;
	}
	
	public CANTalon getAccumulator()
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
	
	public CANTalon getVerticalMotor()
	{
		return verticalMotor;
	}
	
	public Solenoid getLoaderX()
	{
		return loaderX;
	}
	
	public Solenoid getLoaderY()
	{
		return loaderY;
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
			accumulator.set(-0.5);
			rightMotor.set(-0.5);
			leftMotor.set(-0.5);
		}
		else
		{
			accumulator.set(0);
		}
		
	}
}
