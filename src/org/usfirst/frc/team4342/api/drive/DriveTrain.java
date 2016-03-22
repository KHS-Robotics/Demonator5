package org.usfirst.frc.team4342.api.drive;

import edu.wpi.first.wpilibj.CANTalon;
import edu.wpi.first.wpilibj.CANTalon.TalonControlMode;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.PIDController;
import edu.wpi.first.wpilibj.PIDOutput;
import edu.wpi.first.wpilibj.PIDSourceType;

public class DriveTrain implements PIDOutput
{
	private CANTalon fr, fl, mr, ml, rr, rl;
	private PIDController pid;
	
	private double direction;
	
	public DriveTrain(Joystick j, CANTalon fr, CANTalon fl, CANTalon mr,
					  CANTalon ml, CANTalon rr, CANTalon rl)
	{
		this.fr = fr;
		this.fl = fl;
		this.mr = mr;
		this.ml = ml;
		this.rr = rr;
		this.rl = rl;
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
	
	public void setDirection(double direction)
	{
		this.direction = direction;
	}
	
	public double getDirection()
	{
		return direction;
	}
	
	public void setVoltageRampRate(double rampRate)
	{
		for(CANTalon talon : getDriveTrain())
		{
			talon.setVoltageRampRate(rampRate);
		}
	}
	
	public void setBrakeMode()
	{
		for(CANTalon talon : getDriveTrain())
		{
			talon.enableBrakeMode(true);
		}
	}
	
	public void setCoastMode()
	{
		for(CANTalon talon : getDriveTrain())
		{
			talon.enableBrakeMode(false);
		}
	}
	
	public void setMode(TalonControlMode mode)
	{
		for(CANTalon talon : getDriveTrain())
		{
			talon.changeControlMode(mode);
		}
	}
	
	public void enable()
	{
		for(CANTalon talon : getDriveTrain())
		{
			talon.enable();
		}
	}
	
	public void setPIDSourceType (PIDSourceType type)
	{
		for(CANTalon talon : getDriveTrain())
		{
			talon.setPIDSourceType(type);
		}
	}
	
	public void setPIDController(PIDController pid)
	{
		this.pid = pid;
	}
	
	public PIDController getPIDController()
	{
		return pid;
	}
	
	public void stopAll()
	{
		for(CANTalon talon : getDriveTrain())
		{
			talon.set(0);
		}
	}
	
	public CANTalon getFrontRight()
	{
		return fr;
	}
	
	public CANTalon getFrontLeft()
	{
		return fl;
	}
	
	public CANTalon getMiddleRight()
	{
		return mr;
	}
	
	public CANTalon getMiddleLeft()
	{
		return ml;
	}
	
	public CANTalon getRearRight()
	{
		return rr;
	}
	
	public CANTalon getRearLeft()
	{
		return rl;
	}
	
	public CANTalon[] getDriveTrain()
	{
		return new CANTalon[] {
			fr, fl, mr, ml, rr, rl
		};
	}
}
