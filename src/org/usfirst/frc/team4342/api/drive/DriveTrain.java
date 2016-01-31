package org.usfirst.frc.team4342.api.drive;

import edu.wpi.first.wpilibj.CANTalon;
import edu.wpi.first.wpilibj.CANTalon.TalonControlMode;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.PIDController;

public class DriveTrain 
{
	private CANTalon fr, fl, mr, ml, rr, rl;
	public PIDController angleControl;
	
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
