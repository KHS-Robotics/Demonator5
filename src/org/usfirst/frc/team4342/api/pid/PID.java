package org.usfirst.frc.team4342.api.pid;

import org.usfirst.frc.team4342.robot.components.Repository;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.PIDController;
import edu.wpi.first.wpilibj.PIDSource;

public class PID 
{
	private static PIDController angleControl;
	private double pGain = 1, iGain = 0.0, dGain = 0.0;
	private static AHRS navX;

	public PID()
	{
		angleControl = new PIDController(pGain, iGain, dGain, (PIDSource)navX, Repository.TankDrive);

		angleControl.setContinuous();
		angleControl.setInputRange(0.0, 360.0);
		angleControl.setOutputRange(-1.0, 1.0);
		angleControl.enable();
	}
	
	public void pidOn()
	{
		angleControl.enable();
		Repository.TankDrive.pidWrite(angleControl.get());
	}
	public void pidOff()
	{
		angleControl.disable();
	}
	public void setSetpoint(double angle)
	{
		angleControl.setSetpoint(angle);
	}

}

