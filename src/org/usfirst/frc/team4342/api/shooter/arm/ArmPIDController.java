package org.usfirst.frc.team4342.api.shooter.arm;

import edu.wpi.first.wpilibj.PIDController;
import edu.wpi.first.wpilibj.PIDOutput;
import edu.wpi.first.wpilibj.PIDSource;

public final class ArmPIDController extends PIDController 
{
	private double kP, kI, kD;
	private double kPd, kId, kDd;
	
	public ArmPIDController(double Kp, double Ki, double Kd, double Kpd, double Kid, double Kdd, 
						PIDSource source, PIDOutput output, double period) 
	{
		super(Kp, Ki, Kd, source, output, period);
		
		this.kP = Kp;
		this.kI = Ki;
		this.kD = Kd;
		this.kPd = Kpd;
		this.kId = Kid;
		this.kDd = Kdd;
	}
	
	@Override
	public void calculate()
	{
		double input;
		synchronized(this) 
		{
	        input = super.m_pidInput.pidGet();
		}
		
		double error = super.getSetpoint() - input;
		
		if(error > 0)
		{
			super.setPID(kP, kI, kD);
			super.calculate();
		}
		else if(error < 0)
		{
			super.setPID(kPd, kId, kDd);
			super.calculate();
		}
	}
	
	public synchronized void setPUp(double p)
	{
		super.setPID(p, kI, kD);
	}
	
	public synchronized double getPUp()
	{
		return kP;
	}

	public synchronized void setIUp(double i)
	{
		super.setPID(kP, i, kD);
	}
	
	public synchronized double getIUp()
	{
		return kI;
	}
	
	public synchronized void setDUp(double d)
	{
		super.setPID(kP, kI, d);
	}
	
	public synchronized double getDUp()
	{
		return kD;
	}
	
	public synchronized void setPDown(double p)
	{
		this.kPd = p;
	}
	
	public synchronized double getPDown()
	{
		return kPd;
	}
	
	public synchronized void setIDown(double i)
	{
		this.kId = i;
	}
	
	public synchronized double getIDown()
	{
		return kId;
	}
	
	public synchronized void setDDown(double d)
	{
		this.kDd = d;
	}

	public synchronized double getDDown()
	{
		return kDd;
	}
}
