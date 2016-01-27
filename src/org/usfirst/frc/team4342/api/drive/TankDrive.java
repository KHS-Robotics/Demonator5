package org.usfirst.frc.team4342.api.drive;

import org.usfirst.frc.team4342.api.logging.ExceptionInfo;
import org.usfirst.frc.team4342.robot.components.Repository;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.CANTalon;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Joystick;

public class TankDrive 
{
	private static Joystick j;
	private static CANTalon fr, fl, mr, ml, rr, rl;
	private static AHRS navX;
	private static DoubleSolenoid shifter;
	
	private static boolean run;
	
	private TankDrive(Joystick j, DriveTrain talons, AHRS navX, DoubleSolenoid shifter)
	{
		TankDrive.j = j;
		
		TankDrive.fr = talons.getFrontRight();
		TankDrive.fl = talons.getFrontLeft();
		TankDrive.mr = talons.getMiddleRight();
		TankDrive.ml = talons.getMiddleLeft();
		TankDrive.rr = talons.getRearRight();
		TankDrive.rl = talons.getRearLeft();
		
		TankDrive.navX = navX;
		
		TankDrive.shifter = shifter;
	}
	
	public static void startAutomaticMode(Joystick j, DriveTrain talons, AHRS navX, DoubleSolenoid shifter, int shiftButton)
	{
		if(run)
			return;
		
		TankDrive drive = new TankDrive(j, talons, navX, shifter);
		
		run = true;
		
		Thread t = new Thread(new Runnable()
		{
			@Override
			public void run()
			{
				while(run)
				{
					try
					{
						if(DriverStation.getInstance().isEnabled() && DriverStation.getInstance().isOperatorControl())
						{
							drive.drive(shiftButton);
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
	
	public static void stopAutomaticMode()
	{
		run = false;
	}
	
	private void drive(int shiftButton)
	{
		checkUserShift(shiftButton);
		
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
}
