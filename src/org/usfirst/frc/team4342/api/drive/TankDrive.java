package org.usfirst.frc.team4342.api.drive;

import org.usfirst.frc.team4342.robot.components.Repository;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.CANTalon;
import edu.wpi.first.wpilibj.Joystick;

public class TankDrive 
{
	private Joystick j;
	private CANTalon fr, fl, mr, ml, rr, rl;
	private AHRS navX;
	
	public TankDrive(Joystick j, CANTalon fr, CANTalon fl, CANTalon mr,
					CANTalon ml, CANTalon rr, CANTalon rl, AHRS navX)
	{
		this.j = j;
		
		this.fr = fr;
		this.fl = fl;
		this.mr = mr;
		this.ml = ml;
		this.rr = rr;
		this.rl = rl;
		
		this.navX = navX;
	}
	
	public void drive()
	{
		double x = j.getX();
		double y = j.getY();
		double z = j.getZ();
		
		double a;
		
		if (z == 0)
			a = x;
		else if (x == 0)
			a = z;
		else
			a = (x+y) / 2;
		
		double left = (x-a);
		double right = (x+a);
		
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
}
