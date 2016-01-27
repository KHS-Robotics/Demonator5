package org.usfirst.frc.team4342.api.drive;

import org.usfirst.frc.team4342.api.logging.ExceptionInfo;
import org.usfirst.frc.team4342.robot.components.Repository;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.CANTalon;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.Solenoid;

public class TankDrive 
{
	private Joystick j;
	private CANTalon fr, fl, mr, ml, rr, rl;
	private AHRS navX;
	private DoubleSolenoid shifter;
	private Solenoid cylinder;
	
	public TankDrive(Joystick j, CANTalon[] talons, AHRS navX, 
					DoubleSolenoid shifter, Solenoid cylinder)
	{
		this.j = j;
		
		this.fr = talons[0];
		this.fl = talons[1];
		this.mr = talons[2];
		this.ml = talons[3];
		this.rr = talons[4];
		this.rl = talons[5];
		
		this.navX = navX;
		
		this.shifter = shifter;
		this.cylinder = cylinder;
	}
	
	public static void setAutomaticMode(TankDrive drive, int shiftButton, int extendButton)
	{
		Thread t = new Thread(new Runnable()
		{
			@Override
			public void run()
			{
				while(true)
				{
					try
					{
						if(DriverStation.getInstance().isEnabled() && DriverStation.getInstance().isOperatorControl())
						{
							drive.drive(shiftButton, extendButton);
						}
						
						Thread.sleep(50);
					}
					catch(Exception ex)
					{
						Repository.Logs.error(
							"Unexpected error in DriveTrain (" + ExceptionInfo.getType(ex) + ")", 
							ex
						);
					}
				}
			}
		});
		
		t.start();
	}
	
	public void drive(int shiftButton, int extendButton)
	{
		checkUserShift(shiftButton);
		checkUserExtend(extendButton);
		
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
	
	public void setCoastMode()
	{
		fr.enableBrakeMode(false);
		fl.enableBrakeMode(false);
		mr.enableBrakeMode(false);
		ml.enableBrakeMode(false);
		rr.enableBrakeMode(false);
		rl.enableBrakeMode(false);
	}
	
	public void setBrakeMode()
	{
		fr.enableBrakeMode(true);
		fl.enableBrakeMode(true);
		mr.enableBrakeMode(true);
		ml.enableBrakeMode(true);
		rr.enableBrakeMode(true);
		rl.enableBrakeMode(true);
	}
	
	private void checkUserShift(int button)
	{
		if(j.getRawButton(button))
		{
			if(shifter.get() == DoubleSolenoid.Value.kForward)
			{
				shifter.set(DoubleSolenoid.Value.kReverse);
			}
			else
			{
				shifter.set(DoubleSolenoid.Value.kForward);
			}
		}
	}
	
	private void checkUserExtend(int button)
	{
		if(j.getRawButton(button))
		{
			if(cylinder.get())
			{
				cylinder.set(false);
			}
			else
			{
				cylinder.set(true);
			}
		}
	}
}
