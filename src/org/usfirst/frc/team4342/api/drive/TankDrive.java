package org.usfirst.frc.team4342.api.drive;

import org.usfirst.frc.team4342.robot.components.Repository;

import edu.wpi.first.wpilibj.CANTalon;
import edu.wpi.first.wpilibj.Joystick;

public class TankDrive 
{
	private Joystick j1;
	
	private CANTalon fr, fl, rr, rl;
	
	public TankDrive(Joystick j, CANTalon fr, CANTalon fl, CANTalon rr, CANTalon rl)
	{
		this.j1 = j;
		
		this.fr = fr;
		this.fl = fl;
		this.rr = rr;
		this.rl = rl;
	}
	
	public void drive()
	{
		double x = j1.getX();
		double y = j1.getY();
		double z = j1.getZ();
		
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
			rr.set(right);
			rl.set(left);
		}
		catch (Exception ex)
		{
			Repository.logs.error("Failed to set drive motors", ex);
		}
	}
	
}
