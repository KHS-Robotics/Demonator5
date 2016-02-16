package org.usfirst.frc.team4342.api.drive;

import org.usfirst.frc.team4342.api.drive.pid.DrivePID;
import org.usfirst.frc.team4342.robot.components.Repository;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.CANTalon;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.PIDController;
import edu.wpi.first.wpilibj.PIDOutput;
import edu.wpi.first.wpilibj.PIDSourceType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class TankDrive implements PIDOutput
{
	private static final double JOYSTICK_SENSITIVITY = 0.9;
	
	private Joystick j;
	private DriveTrain driveTrain;
	private CANTalon fr, fl, mr, ml, rr, rl;
	private AHRS navX;
	private DoubleSolenoid shifter;
	private Encoder encLeft, encRight;
	
	private PIDController angleControl;
	private double direction;
	private boolean firstRunPID, firstRunGoStraight;
	
	private PIDController frPID, mrPID, rrPID;
	private PIDController flPID, mlPID, rlPID;
	
	private boolean autoStepFinished;
	
	public TankDrive(Joystick j, DriveTrain talons, AHRS navX, DoubleSolenoid shifter, 
					Encoder encLeft, Encoder encRight)
	{
		this.j = j;
		
		this.driveTrain = talons;
		this.driveTrain.setPIDSourceType(PIDSourceType.kRate);
		fr = driveTrain.getFrontRight();
		fl = driveTrain.getFrontLeft();
		mr = driveTrain.getMiddleRight();
		ml = driveTrain.getMiddleLeft();
		rr = driveTrain.getRearRight();
		rl = driveTrain.getRearLeft();
		
		this.navX = navX;
		
		this.shifter = shifter;
		
		this.encRight = encRight;
		this.encLeft = encLeft;
		
		frPID = new PIDController(DrivePID.Right.kP, DrivePID.Right.kI, DrivePID.Right.kD, encRight, this.fr);
		frPID.setContinuous();
		frPID.setInputRange(-1.0, 1.0);
		frPID.setOutputRange(-1.0, 1.0);
		frPID.enable();
		
		mrPID = new PIDController(DrivePID.Right.kP, DrivePID.Right.kI, DrivePID.Right.kD, encRight, this.mr);
		mrPID.setContinuous();
		mrPID.setInputRange(-1.0, 1.0);
		mrPID.setOutputRange(-1.0, 1.0);
		mrPID.enable();

		rrPID = new PIDController(DrivePID.Right.kP, DrivePID.Right.kI, DrivePID.Right.kD, encRight, this.rr);
		rrPID.setContinuous();
		rrPID.setInputRange(-1.0, 1.0);
		rrPID.setOutputRange(-1.0, 1.0);
		rrPID.enable();
		
		flPID = new PIDController(DrivePID.Left.kP, DrivePID.Left.kI, DrivePID.Left.kD, encLeft, this.fl);
		flPID.setContinuous();
		flPID.setInputRange(-1.0, 1.0);
		flPID.setOutputRange(-1.0, 1.0);
		flPID.enable();
		
		mlPID = new PIDController(DrivePID.Left.kP, DrivePID.Left.kI, DrivePID.Left.kD, encLeft, this.ml);
		mlPID.setContinuous();
		mlPID.setInputRange(-1.0, 1.0);
		mlPID.setOutputRange(-1.0, 1.0);
		mlPID.enable();
		
		rlPID = new PIDController(DrivePID.Left.kP, DrivePID.Left.kI, DrivePID.Left.kD, encLeft, this.rl);
		rlPID.setContinuous();
		rlPID.setInputRange(-1.0, 1.0);
		rlPID.setOutputRange(-1.0, 1.0);
		rlPID.enable();
		
		angleControl = new PIDController(DrivePID.Rotational.kP, DrivePID.Rotational.kI, DrivePID.Rotational.kD, navX, this);
		angleControl.setContinuous();
		angleControl.setInputRange(-180.0, 180.0);
		angleControl.setOutputRange(-1.0, 1.0);
		angleControl.disable();
	}
	
	@Override
	public void pidWrite(double output) 
	{
		double right = direction - output;
		double left = direction + output;
		
		if (right > 1)
			right = 1; 
		if (left > 1)
			left = 1;
		if (right < -1)
			right = -1; 
		if (left < -1)
			left = -1;	
		
		frPID.setSetpoint(right);
		flPID.setSetpoint(left);
		mrPID.setSetpoint(right);
		mlPID.setSetpoint(left);
		rrPID.setSetpoint(right);
		rlPID.setSetpoint(left);
	}
	
	public synchronized void drive(int shiftButton)
	{
		if(Repository.SwitchBox.getRawButton(4) && !firstRunPID)
		{
			setPID(
				SmartDashboard.getNumber("Drive-P"),
				SmartDashboard.getNumber("Drive-I"),
				SmartDashboard.getNumber("Drive-D")
			); 
			
			goToAngle(0);
			turnPIDOn();
			
			firstRunPID = true;
		}
		else if(!Repository.SwitchBox.getRawButton(4))
		{
			firstRunPID = false;
		}
		
		if(Repository.DriveStick.getRawButton(7) && !firstRunGoStraight)
		{
			goToSetpoint(navX.getYaw());
			angleControl.enable();
			
			firstRunGoStraight = true;
		}
		else if(Repository.DriveStick.getRawButton(7) && firstRunGoStraight)
		{
			goStraight();
		}
		else
		{
			joystickDrive(shiftButton);
		}
	}
	
	public void joystickDrive(int shiftButton)
	{
		turnPIDOff();
		checkUserShift(shiftButton);

		double x = sensitivityControl(j.getTwist());
		double y = sensitivityControl(j.getY());

		double left = (y-x);
		double right = (y+x);

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
			frPID.setSetpoint(right);
			flPID.setSetpoint(left);
			mrPID.setSetpoint(right);
			mlPID.setSetpoint(left);
			rrPID.setSetpoint(right);
			rlPID.setSetpoint(left);
		}
		catch (Exception ex)
		{
			Repository.Logs.error("Failed to set drive motors", ex);
		}
	}
	
	public void goStraight()
	{
		setDirection(j.getY());
	}
	
	public void goToAngle(double angle)
	{
		turnPIDOn();
		goToSetpoint(angle);
		setDirection(0.0);
	}
	
	public synchronized void stopAll()
	{
		driveTrain.stopAll();
	}
	
	public synchronized void turnPIDOn()
	{
		angleControl.enable();
	}
	
	public synchronized void turnPIDOff()
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
	
	public void setRightPID(double p, double i, double d)
	{
		frPID.setPID(p, i, d);
		mrPID.setPID(p, i, d);
		rrPID.setPID(p, i, d);
	}
	
	public void setLeftPID(double p, double i, double d)
	{
		rlPID.setPID(p, i, d);
		mlPID.setPID(p, i, d);
		rlPID.setPID(p, i, d);
	}
	
	public synchronized void setDirection(double power)
	{
		direction = power;
	}
	
	public synchronized double getDirection()
	{
		return direction;
	}
	
	public synchronized boolean isAutoStepFinished()
	{
		return autoStepFinished;
	}
	
	private synchronized void checkUserShift(int button)
	{
		if(j.getRawButton(button))
			shifter.set(DoubleSolenoid.Value.kForward);
		else
			shifter.set(DoubleSolenoid.Value.kReverse);	
	}
	
	private double sensitivityControl(double input)
	{
		return (JOYSTICK_SENSITIVITY*Math.pow(input, 3))+((1-input)*JOYSTICK_SENSITIVITY);
	}
}
