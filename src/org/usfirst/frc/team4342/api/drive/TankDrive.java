package org.usfirst.frc.team4342.api.drive;

import org.usfirst.frc.team4342.api.drive.pid.DrivePID;
import org.usfirst.frc.team4342.robot.components.Repository;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.CANTalon;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.PIDController;
import edu.wpi.first.wpilibj.PIDOutput;
import edu.wpi.first.wpilibj.PIDSourceType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import org.usfirst.frc.team4342.api.drive.DefenseStates;

public class TankDrive implements PIDOutput
{
	private static final double JOYSTICK_SENSITIVITY = 0.9;
	
	private Joystick j;
	private DriveTrain driveTrain;
	private CANTalon fr, fl, mr, ml, rr, rl;
	private AHRS navX;
	private DoubleSolenoid shifter;
	private Encoder encLeft, encRight;
	
	private PIDController angleControl;
	private double direction;
	private boolean firstRunPID, firstRunGoStraight = true;
	
	private boolean autoStepFinished;
	private DefenseStates state;
	
	public TankDrive(Joystick j, DriveTrain talons, AHRS navX, DoubleSolenoid shifter, 
					Encoder encLeft, Encoder encRight)
	{
		this.j = j;
		
		this.driveTrain = talons;
		this.driveTrain.setPIDSourceType(PIDSourceType.kRate);
		fr = driveTrain.getFrontRight();
		fl = driveTrain.getFrontLeft();
		mr = driveTrain.getMiddleRight();
		ml = driveTrain.getMiddleLeft();
		rr = driveTrain.getRearRight();
		rl = driveTrain.getRearLeft();
		
		this.navX = navX;
		
		this.shifter = shifter;
		
		this.encRight = encRight;
		this.encLeft = encLeft;
		
		angleControl = new PIDController(DrivePID.Rotational.kP, DrivePID.Rotational.kI, DrivePID.Rotational.kD, navX, this);
		angleControl.setContinuous();
		angleControl.setInputRange(-180.0, 180.0);
		angleControl.setOutputRange(-1.0, 1.0);
		angleControl.setAbsoluteTolerance(2);
		angleControl.disable();
	}
	
	@Override
	public void pidWrite(double output) 
	{
		double right = direction - output;
		double left = direction + output;
		
		if (right > 1)
			right = 1; 
		if (left > 1)
			left = 1;
		if (right < -1)
			right = -1; 
		if (left < -1)
			left = -1;	
		
		fr.set(right);
		rl.set(left);
		mr.set(right);
		ml.set(left);
		rr.set(right);
		rl.set(left);
	}
	
	public synchronized void drive(int shiftButton, int straightButton, int angleButton, int autoForwardButton,
								   int autoReverseButton)
	{
		if(j.getRawButton(straightButton))
		{
			if (firstRunGoStraight)
			{
				goStraight(j.getY(), navX.getYaw());
				
				firstRunGoStraight = false;
			}
			else
				goStraight(j.getY());
		}
		else if(j.getRawButton(angleButton))
		{
			goToAngle(0.0);
		}
		else if(j.getRawButton(autoForwardButton))
		{
			autoRampParts(true, 0.75, true, 50.0);
		}
		else if(j.getRawButton(autoReverseButton))
		{
			autoRampParts(false, 0.75, false, 0.0);
		}
		else
		{
			joystickDrive(shiftButton);
			
			firstRunGoStraight = true;
		}
	}
	
	public void joystickDrive(int shiftButton)
	{
		turnPIDOff();
		checkUserShift(shiftButton);

		double x = sensitivityControl(j.getZ());
		double y = sensitivityControl(-j.getY());

		double left = (y-x);
		double right = (y+x);

		if (left > 1.0)
			left = 1.0;
		else if (left < -1.0)
			left = -1.0;

		if (right > 1.0)
			right = 1.0;
		else if (right < -1.0)
			right = -1.0;

		try
		{
			fr.set(right);
			rl.set(left);
			mr.set(right);
			ml.set(left);
			rr.set(right);
			rl.set(left);
		}
		catch (Exception ex)
		{
			Repository.Logs.error("Failed to set drive motors", ex);
		}
	}
	
	public void autoRampParts(boolean forward, double direction, boolean target, double goalAngle)
	{
		double startAngle = 0.0 + (forward ? 180.0 : 0.0);
		
		if (state == DefenseStates.APPROACH)
		{
			goToAngle(startAngle);
			if(angleControl.onTarget())
				state = DefenseStates.CLIMB;
		}
		else if (state == DefenseStates.CLIMB)
		{
			goStraight(-Math.abs(direction));
			
			if (false)
			{
				state = DefenseStates.PEAK;
			}
		}
		else if (state == DefenseStates.PEAK)
		{
			
			if (false)
			{
				state = DefenseStates.DESCENT;
			}
		}
		else if (state == DefenseStates.DESCENT)
		{
			
			if (false)
			{
				state = DefenseStates.FINISHING;
			}
		}
		else if (state == DefenseStates.FINISHING)
		{
			if(target)
			{
				goToAngle(goalAngle);
				
				if (angleControl.onTarget())
					state = DefenseStates.FINISH;
			}
			else
				state = DefenseStates.FINISH;
			
		}
		else if (state == DefenseStates.FINISH)
		{
			if(target)
			{
				goToAngle(goalAngle);
			}
			else
				goStraight(-Math.abs(direction));
		}
	}
	
	public void goStraight(double direction)
	{
		setDirection(direction);
	}
	
	public void goStraight(double direction, double angle)
	{
		goToSetpoint(angle);
		goStraight(direction);
	}
	
	public void goToAngle(double angle)
	{
		goToSetpoint(angle);
		setDirection(0.0);
	}
	
	public synchronized void stopAll()
	{
		driveTrain.stopAll();
	}
	
	public synchronized void turnPIDOn()
	{
		angleControl.enable();
	}
	
	public synchronized void turnPIDOff()
	{
		angleControl.disable();
	}
	
	public synchronized void goToSetpoint(double setpointAngle)
	{
		angleControl.setSetpoint(setpointAngle);
		turnPIDOn();
	}
	
	public synchronized void setYawPID(double p, double i, double d)
	{
		angleControl.setPID(p, i, d);
	}
	
	public synchronized void setDirection(double power)
	{
		direction = power;
	}
	
	public synchronized double getDirection()
	{
		return direction;
	}
	
	public synchronized boolean isAutoStepFinished()
	{
		return autoStepFinished;
	}
	
	private synchronized void checkUserShift(int button)
	{
		if(j.getRawButton(button))
			shifter.set(DoubleSolenoid.Value.kForward);
		else
			shifter.set(DoubleSolenoid.Value.kReverse);	
	}
	
	private double sensitivityControl(double input)
	{
		return (JOYSTICK_SENSITIVITY*Math.pow(input, 3))+((1-JOYSTICK_SENSITIVITY)*input);
	}
}
