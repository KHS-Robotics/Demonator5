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
import org.usfirst.frc.team4342.api.drive.DefenseState;

public class TankDrive implements PIDOutput
{
	private static final double JOYSTICK_SENSITIVITY = 0.95;
	private static final double DEAD_BAND = 0.08;
	
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
	private DefenseState state;
	private double startingPitch = 0.0, minPitch = 0.0, maxPitch = 0.0, lastPitch = 0.0; 
	private boolean firstRun = true;
	private final double PITCH_WINDOW = 17;
	
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
		
		driveTrain.setPIDController(angleControl);
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
		fl.set(left);
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
				goToSetpoint(navX.getYaw());
				
				firstRunGoStraight = false;
			}
			else
				goStraight(j.getRawAxis(3));
		}
		else if(false)
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
			if (!firstRunGoStraight)
				turnPIDOff();
			joystickDrive(shiftButton);
			
			firstRunGoStraight = true;
		}
	}
	
	public void joystickDrive(int shiftButton)
	{
		checkUserShift(shiftButton);

		double posy=sensitivityControl(j.getRawAxis(3));
		double negy=-sensitivityControl(j.getRawAxis(2));
		double x=sensitivityControl(j.getRawAxis(0));
		
		double y=posy+negy;
		
		try
		{
			fr.set(y-x);
			fl.set(y+x);
			mr.set(y-x);
			ml.set(y+x);
			rr.set(y-x);
			rl.set(y+x);
		}
		catch (Exception ex)
		{
			Repository.Logs.error("Failed to set drive motors", ex);
		}
//		double x = sensitivityControl(-j.getZ());
//		double y = sensitivityControl(-j.getY());
//
//		double left = (y-x);
//		double right = (y+x);
//
//		if (left > 1.0)
//			left = 1.0;
//		else if (left < -1.0)
//			left = -1.0;
//
//		if (right > 1.0)
//			right = 1.0;
//		else if (right < -1.0)
//			right = -1.0;
//
//		try
//		{
//			fr.set(right);
//			fl.set(left);
//			mr.set(right);
//			ml.set(left);
//			rr.set(right);
//			rl.set(left);
//		}
//		catch (Exception ex)
//		{
//			Repository.Logs.error("Failed to set drive motors", ex);
//		}
	}
	
	public void autoRampParts(boolean forward, double direction, boolean target, double goalAngle)
	{
		double startAngle = 0.0 + (forward ? 180.0 : 0.0);
		double currentPitch = navX.getPitch();
		
		if (state == DefenseState.APPROACH)
		{
			goToAngle(startAngle);
			
			if (firstRun)
			{
				startingPitch = navX.getPitch();
				minPitch = navX.getPitch();
				maxPitch = navX.getPitch();
				firstRun = false;
			}
			
			if(currentPitch < minPitch)
				minPitch = currentPitch;
			else if(currentPitch > maxPitch)
				maxPitch = currentPitch;
				
			if(angleControl.onTarget())
				state = DefenseState.CLIMB;
		}
		else if (state == DefenseState.CLIMB)
		{
			goStraight(-Math.abs(direction));
			
			if(currentPitch < minPitch)
				minPitch = currentPitch;
			else if(currentPitch > maxPitch)
				maxPitch = currentPitch;
			
			if (minPitch <= -PITCH_WINDOW && currentPitch > minPitch && currentPitch > lastPitch)
			{
				state = DefenseState.PEAK;
			}
		}
		else if (state == DefenseState.PEAK)
		{
			if(currentPitch < minPitch)
				minPitch = currentPitch;
			else if(currentPitch > maxPitch)
				maxPitch = currentPitch;
			
			if (maxPitch >= PITCH_WINDOW && currentPitch < maxPitch && currentPitch < lastPitch)
			{
				state = DefenseState.DESCENT;
			}
		}
		else if (state == DefenseState.DESCENT)
		{
			if(currentPitch < minPitch)
				minPitch = currentPitch;
			else if(currentPitch > maxPitch)
				maxPitch = currentPitch;
			
			if ((currentPitch - startingPitch) < 10)
			{
				state = DefenseState.FINISHING;
			}
		}
		else if (state == DefenseState.FINISHING)
		{
			
			if(target)
			{
				goToAngle(goalAngle);
				
				if (angleControl.onTarget())
					state = DefenseState.FINISH;
			}
			else
				state = DefenseState.FINISH;
			
		}
		else if (state == DefenseState.FINISH)
		{
			if(target)
				goToAngle(goalAngle);
			else
				goStraight(-Math.abs(direction));
		}
		
		lastPitch = currentPitch;
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
		if(angleControl.isEnabled())
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
			shifter.set(DoubleSolenoid.Value.kReverse);
		else
			shifter.set(DoubleSolenoid.Value.kForward);	
	}
	
	private double sensitivityControl(double input)
	{
		if(Math.abs(input)<DEAD_BAND){
			input=0;
		}
		return (JOYSTICK_SENSITIVITY*Math.pow(input, 7))+((1-JOYSTICK_SENSITIVITY)*input);
	}
}
