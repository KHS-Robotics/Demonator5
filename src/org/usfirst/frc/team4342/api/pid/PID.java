package org.usfirst.frc.team4342.api.pid;

import org.usfirst.frc.team4342.robot.components.Repository;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class PID 
{
	private static PIDController angleControl;
	private double pGain = 1, iGain = 0.0, dGain = 0.0;
	private static AHRS navX;

	public PID()
	{
		pGain = Integer.parseInt(SmartDashboard.getData("Drive P Value").toString());
		iGain = Integer.parseInt(SmartDashboard.getData("Drive I Value").toString());
		dGain = Integer.parseInt(SmartDashboard.getData("Drive D Value").toString());
		angleControl = new PIDController(pGain, iGain, dGain, navX, Repository.TankDrive);
		
		angleControl.setContinuous();
		angleControl.setInputRange(0.0, 360.0);
		angleControl.setOutputRange(-1.0, 1.0);
		angleControl.enable();
	}
	
	public void turnOn()
	{
		angleControl.enable();
		Repository.TankDrive.pidWrite(angleControl.get());
	}
	
	public void turnOff()
	{
		angleControl.disable();
	}
	
	public void setSetpoint(double angle)
	{
		angleControl.setSetpoint(angle);
	}
	
	public void setP(double p)
	{
		pGain = p;
	}
	
	public void setI(double i)
	{
		iGain = i;
	}
	
	public void setD(double d)
	{
		dGain = d;
	}
}

