package org.usfirst.frc.team4342.api.shooter.arm.pid;

import org.usfirst.frc.team4342.robot.components.Repository;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.PIDController;
import edu.wpi.first.wpilibj.PIDOutput;
import edu.wpi.first.wpilibj.PIDSource;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public final class ArmPIDController extends PIDController 
{
	private double kP, kI, kD;
	private double kPd, kId, kDd;
	private Encoder enc;
	
	public ArmPIDController(double Kp, double Ki, double Kd, double Kpd, double Kid, double Kdd, 
							Encoder enc, PIDSource source, PIDOutput output, double period) 
	{
		super(Kp, Ki, Kd, source, output, period);
		
		this.kP = Kp;
		this.kI = Ki;
		this.kD = Kd;
		this.kPd = Kpd;
		this.kId = Kid;
		this.kDd = Kdd;
		this.enc = enc;
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
		
		SmartDashboard.putNumber("Arm-Error", error);
		SmartDashboard.putNumber("Arm-PidGet", input);
		SmartDashboard.putNumber("Arm-Val", Repository.ArmMotor.get());
		
		if(error > 0)
		{
			if(enc.get() < 140 && this.getSetpoint() > 140)
			{
				super.setPID(kP, kI, kD);
				super.calculate();
			}
			else
			{
				super.setPID(kPd, kId, kDd);
				super.calculate();
			}
		}
		else if(error < 0)
		{
			if(enc.get() < 140 && this.getSetpoint() < 140)
			{
				super.setPID(0, 0, 0);
				super.calculate();
			}
			else
			{
				super.setPID(kP, kI, kD);
				super.calculate();
			}
		}
	}
	
	public synchronized void setPIDUp(double p, double i, double d)
	{
		kP = p;
		kI = i;
		kD = d;
	}
	
	public synchronized void setPIDDown(double p, double i, double d)
	{
		kPd = p;
		kId = i;
		kDd = d;
	}
	
	public synchronized void setPUp(double p)
	{
		kP = p;
	}

	public synchronized void setIUp(double i)
	{
		kI = i;
	}
	
	public synchronized void setDUp(double d)
	{
		kD = d;
	}
	
	public synchronized void setPDown(double p)
	{
		kPd = p;
	}
	
	public synchronized void setIDown(double i)
	{
		kId = i;
	}
	
	public synchronized void setDDown(double d)
	{
		kDd = d;
	}
	
	public synchronized double getPUp()
	{
		return kP;
	}
	
	public synchronized double getIUp()
	{
		return kI;
	}
	
	public synchronized double getDUp()
	{
		return kD;
	}
	
	public synchronized double getPDown()
	{
		return kPd;
	}
	
	public synchronized double getIDown()
	{
		return kId;
	}
	
	public synchronized double getDDown()
	{
		return kDd;
	}
}
