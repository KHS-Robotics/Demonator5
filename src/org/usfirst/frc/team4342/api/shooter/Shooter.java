package org.usfirst.frc.team4342.api.shooter;

import org.usfirst.frc.team4342.api.arm.ArmController;

public class Shooter 
{
	public static final double JOYSTICK_DEADBAND = 0.05;
	
	//TODO: angle -> encoder distance
	public static final double LOW_BAR_SHOOT_ANGLE = 27, LOW_BAR_ENC_DIST = 400;
	public static final double HIGH_BATTER_ANGLE = 63.67, LOW_BATTER_ANGLE = 32;
	public static final double THIRD_POSITION_HIGH_ANGLE = 27, THIRD_POSITION_LOW_DIST = 350;
	public static final double FOURTH_POSITION_HIGH_DIST = 300, FOURTH_POSITION_LOW_DIST = 400;

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

	public void handleTeleop(int driverShootButton, int safetyButton, int fireButton, int smartDashboardButton, 
							int armBrakeButton, int accumInButton, int accumOutButton, int accumLiftButton)
	{
		shooter.checkUser(driverShootButton, safetyButton, fireButton, accumInButton, accumOutButton);
		arm.checkUser(smartDashboardButton, armBrakeButton, accumInButton, accumOutButton);
	}

	public void handleAuto()
	{
		arm.setSetpoint(autoArmSetpoint);
		shooter.setMotorsPID(autoShooterSetpoint);
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
