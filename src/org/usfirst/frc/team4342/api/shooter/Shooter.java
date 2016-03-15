package org.usfirst.frc.team4342.api.shooter;

import org.usfirst.frc.team4342.api.arm.ArmController;

public class Shooter 
{
	public static final double JOYSTICK_DEADBAND = 0.05;
	public static final double BATTER_ANGLE = 63.67, LOW_BAR_SHOOT_ANGLE = 27; //TODO: angle -> encoder distance
	public static final double LOW_BAR_ENC_DIST = 400;

	private ShooterController shooter;
	private ArmController arm;

	private boolean ballPushStatus;
	private double autoShooterSetpoint;
	private double autoArmSetpoint;

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
		arm.setSetpoint(autoArmSetpoint);
		shooter.setMotorsPID(autoShooterSetpoint);
		shooter.setBallPusher(ballPushStatus);
	}

	public void setArmSetpoint(double setpoint)
	{
		this.autoArmSetpoint = setpoint;
	}
	
	public void setShooterMotorsPID(double setpoint)
	{
		this.autoShooterSetpoint = setpoint;
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
