package org.usfirst.frc.team4342.api.shooter;

import org.usfirst.frc.team4342.api.shooter.arm.ArmController;

public class Shooter 
{
	public static final double JOYSTICK_DEADBAND = 0.05;
	
	public static final double LOW_BAR_SHOOT_ANGLE = 27, LOW_BAR_ENC_DIST = 450;
	public static final double HIGH_BATTER_DIST = 280, LOW_GOAL_DIST = 395;
	public static final double THIRD_POSITION_HIGH_DIST = 347.75, THIRD_POSITION_LOW_DIST = 350;
	public static final double FOURTH_POSITION_HIGH_DIST = 325, FOURTH_POSITION_LOW_DIST = 375;

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

	public void handleTeleop(int highGoalSafetyButton, int lowGoalSafetyButton, int fireButton, int smartDashboardButton, int accumInButton, int accumOutButton)
	{
		shooter.checkUser(highGoalSafetyButton, lowGoalSafetyButton, fireButton, accumInButton, accumOutButton);
		arm.checkUser(smartDashboardButton, accumInButton, accumOutButton);
	}

	public void handleAuto()
	{
		arm.setSetpoint(autoArmSetpoint);
		shooter.setSetpoint(autoShooterSetpoint);
		shooter.setBallPusher(ballPushStatus);
	}
	
	public void disableShooterPID()
	{
		shooter.disablePID();
	}

	public void setArmSetpoint(double setpoint)
	{
		this.autoArmSetpoint = setpoint;
	}
	
	public void setShooterSetpoint(double setpoint)
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
