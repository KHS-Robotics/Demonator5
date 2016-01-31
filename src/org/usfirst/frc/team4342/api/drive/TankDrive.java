package org.usfirst.frc.team4342.api.drive;

import org.usfirst.frc.team4342.robot.components.Repository;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.CANTalon;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.Joystick;

public class TankDrive 
{
	private static Joystick j;
	private DriveTrain driveTrain;
	private static CANTalon fr, fl, mr, ml, rr, rl;
	private static AHRS navX;
	private static DoubleSolenoid shifter;
	
	public TankDrive(Joystick j, DriveTrain talons, AHRS navX, DoubleSolenoid shifter)
	{
		TankDrive.j = j;
		
		driveTrain = talons;
		fr = driveTrain.getFrontRight();
		fl = driveTrain.getFrontLeft();
		mr = driveTrain.getMiddleRight();
		ml = driveTrain.getMiddleLeft();
		rr = driveTrain.getRearRight();
		rl = driveTrain.getRearLeft();
		
		TankDrive.navX = navX;
		
		TankDrive.shifter = shifter;
	}
	
	public void drive(int shiftButton)
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
			a = (x+z) / 2.0;
		
		double left = (y-a);
		double right = (y+a);
		
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
	
	public void stopAll()
	{
		driveTrain.stopAll();
	}
	
	private void checkUserShift(int button)
	{
		if(j.getRawButton(button))
			shifter.set(DoubleSolenoid.Value.kReverse);
		else
			shifter.set(DoubleSolenoid.Value.kForward);	
	}
}
