package org.usfirst.frc.team4342.api.shooter;

import org.usfirst.frc.team4342.api.shooter.controllers.ArmController;
import org.usfirst.frc.team4342.api.shooter.controllers.ShooterController;

public class Shooter 
{
	public static final double JOYSTICK_DEADBAND = 0.05;
	
	private ShooterController shooter;
	private ArmController arm;
	
	private boolean accumStatus, ballPushStatus;
	private double autoMotorOutput;
	private int autoSetpoint;
	
	public Shooter(ShooterController shooter, ArmController arm)
	{
		this.shooter = shooter;
		this.arm = arm;
	}
	
	public void handleTeleop(int safetyButton, int fireButton, int armBrakeButton, int accumButton, int accumLiftButton)
	{
		shooter.checkUser(safetyButton, fireButton, accumButton);
		arm.checkUser(armBrakeButton, accumButton, accumLiftButton, safetyButton);
	}
	
	public void handleAuto()
	{
		arm.setSetpoint(autoSetpoint);
		arm.setAccumLifter(accumStatus);
		shooter.setMotors(autoMotorOutput);
		shooter.setBallPusher(ballPushStatus);
	}
	
	public void setArmSetpoint(int setpoint)
	{
		this.autoSetpoint = setpoint;
	}
	
	public void setAccumulatorLifter(boolean on)
	{
		this.accumStatus = on;
	}
	
	public void setShooterMotors(double output)
	{
		this.autoMotorOutput = output;
	}
	
	public void setBallPusher(boolean on)
	{
		this.ballPushStatus = on;
	}
	
	public void stopAll()
	{
		shooter.stopAll();
		arm.stopAll();
	}
}
