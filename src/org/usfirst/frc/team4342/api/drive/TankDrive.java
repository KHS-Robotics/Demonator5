package org.usfirst.frc.team4342.api.drive;

import org.usfirst.frc.team4342.robot.components.Repository;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.CANTalon;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.PIDController;
import edu.wpi.first.wpilibj.PIDOutput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class TankDrive implements PIDOutput
{
	private Joystick j;
	private DriveTrain driveTrain;
	private CANTalon fr, fl, mr, ml, rr, rl;
	private AHRS navX;
	private DoubleSolenoid shifter;
	private PIDController angleControl;
	private double direction = 0;
	private boolean autoStepFinished;
	private Encoder encLeft, encRight;
	private static boolean lastButton4, lastButton7;
	private static final double DEAD_BAND = 0.05;
	private double pGain = 1.0, iGain = 0.0, dGain = 0.0;
	
	
	public TankDrive(Joystick j, DriveTrain talons, AHRS navX, DoubleSolenoid shifter, Encoder encLeft,
					 Encoder encRight)
	{
		this.j = j;
		
		this.driveTrain = talons;
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
		
		SmartDashboard.putNumber("Drive-P", pGain);
		SmartDashboard.putNumber("Drive-I", iGain);
		SmartDashboard.putNumber("Drive-D", dGain);
		angleControl = new PIDController(pGain, iGain, dGain, navX, this, 20);
		
		angleControl.setContinuous();
		angleControl.setInputRange(-180.0, 180.0);
		angleControl.setOutputRange(-1.0, 1.0);
		angleControl.disable();
	}
	
	public synchronized boolean isAutoStepFinished()
	{
		return autoStepFinished;
	}
	
	public void goStraight(boolean firstRun)
	{
		if (true == firstRun)
		{
			goToSetpoint(navX.getYaw());
			angleControl.enable();
		}
		
		setDirection(j.getY());
	}
	
	public void goToAngle(double angle)
	{
		turnPIDOn();
		goToSetpoint(angle);
		setDirection(0.0);
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
	
	public synchronized void setDirection(double power)
	{
		direction = power;
	}
	
	public synchronized double getDirection()
	{
		return direction;
	}
	
	public synchronized void drive()
	{
		if (lastButton4 != Repository.SwitchBox.getRawButton(4))
		{
			setPID(
				SmartDashboard.getNumber("Drive-P"),
				SmartDashboard.getNumber("Drive-I"),
				SmartDashboard.getNumber("Drive-D")
			); 
			
			goToSetpoint(0);
			turnPIDOn();
		}
		else if (Repository.DriveStick.getRawButton(7)) 
		{
			Repository.TankDrive.goStraight(Repository.DriveStick.getRawButton(7) == lastButton7);
		}
		else
		{
			joystickDrive();
		}	
		
		lastButton4 = Repository.SwitchBox.getRawButton(4);
		lastButton7 = Repository.DriveStick.getRawButton(7);
	}
	
	public void joystickDrive()
	{
		turnPIDOff();
		checkUserShift();

		double z = Math.abs(j.getZ()) > DEAD_BAND ? -j.getZ() : 0.0;
		double y = Math.abs(j.getY()) > DEAD_BAND ? -j.getY() : 0.0;

		double left = (y-z);
		double right = (y+z);

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
			fl.set(left);
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
	
	public synchronized void stopAll()
	{
		driveTrain.stopAll();
	}
	
	private synchronized void checkUserShift()
	{
		if(j.getRawButton(9))
			shifter.set(DoubleSolenoid.Value.kForward);
		else
			shifter.set(DoubleSolenoid.Value.kReverse);	
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
	}
	
	public synchronized void setPID(double p, double i, double d)
	{
		pGain = p;
		iGain = i;
		dGain = d;
		angleControl.setPID(p, i, d);
	}
}
