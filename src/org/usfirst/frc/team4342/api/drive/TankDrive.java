package org.usfirst.frc.team4342.api.drive;

import org.usfirst.frc.team4342.robot.components.Repository;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.CANTalon;
import edu.wpi.first.wpilibj.DoubleSolenoid;
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
	
	public TankDrive(Joystick j, DriveTrain talons, AHRS navX, DoubleSolenoid shifter)
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
		
		int pGain = Integer.parseInt(SmartDashboard.getData("Drive P Value").toString());
		int iGain = Integer.parseInt(SmartDashboard.getData("Drive I Value").toString());
		int dGain = Integer.parseInt(SmartDashboard.getData("Drive D Value").toString());
		angleControl = new PIDController(pGain, iGain, dGain, navX, Repository.TankDrive);
		
		angleControl.setContinuous();
		angleControl.setInputRange(0.0, 360.0);
		angleControl.setOutputRange(-1.0, 1.0);
		angleControl.enable();
	}
	
	@Override
	public void pidWrite(double output) 
	{
		fr.set(-output);
		fl.set(output);
		mr.set(-output);
		ml.set(output);
		rr.set(-output);
		rl.set(output);
	}
	
	public synchronized void drive(int shiftButton)
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
	
	public synchronized void stopAll()
	{
		driveTrain.stopAll();
	}
	
	private synchronized void checkUserShift(int button)
	{
		if(j.getRawButton(button))
			shifter.set(DoubleSolenoid.Value.kReverse);
		else
			shifter.set(DoubleSolenoid.Value.kForward);	
	}
	
	public synchronized void turnOn()
	{
		angleControl.enable();
		Repository.TankDrive.pidWrite(angleControl.get());
	}
	
	public synchronized void turnOff()
	{
		angleControl.disable();
	}
	
	public synchronized void goToSetpoint(double setpointAngle)
	{
		angleControl.setSetpoint(setpointAngle);
	}
	
	public synchronized void setPID(double p, double i, double d)
	{
		angleControl.setPID(p, i, d);
	}
}
