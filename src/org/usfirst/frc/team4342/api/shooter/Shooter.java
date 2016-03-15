package org.usfirst.frc.team4342.api.shooter;

import org.usfirst.frc.team4342.api.arm.ArmController;

public class Shooter 
{
	public static final double JOYSTICK_DEADBAND = 0.05;

	private ShooterController shooter;
	private ArmController arm;

	private boolean ballPushStatus;
	private double autoMotorOutput;
	private int autoSetpoint;

	public Shooter(ShooterController shooter, ArmController arm)
	{
		this.shooter = shooter;
		this.arm = arm;
	}

	public void handleTeleop(int driverShootButton, int safetyButton, int fireButton, int smartDashboardButton, int armBrakeButton, int accumButton, int accumLiftButton)
	{
		shooter.checkUser(driverShootButton, safetyButton, fireButton, accumButton);
		arm.checkUser(smartDashboardButton, armBrakeButton, accumButton);
	}

	public void handleAuto()
	{
		arm.setSetpoint(autoSetpoint);
		shooter.setMotorsPID(autoMotorOutput);
		shooter.setBallPusher(ballPushStatus);
	}

	public void setArmSetpoint(int setpoint)
	{
		this.autoSetpoint = setpoint;
	}
	
	public void setShooterMotorsPID(double output)
	{
		this.autoMotorOutput = output;
	}

	public void setBallPusher(boolean on)
	{
		this.ballPushStatus = on;
	}
	
	public boolean armIsAtSetpoint()
	{
		return arm.isAtAutoSetpoint();
	}
	
	public boolean shooterIsAtSetpoint()
	{
		return shooter.isAtSetpoint();
	}

	public void stopAll()
	{
		shooter.stopAllMotors();
		arm.stopAll();
	}
}
