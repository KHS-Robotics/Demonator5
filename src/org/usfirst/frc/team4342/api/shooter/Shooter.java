package org.usfirst.frc.team4342.api.shooter;

import org.usfirst.frc.team4342.api.arm.ArmController;
import org.usfirst.frc.team4342.api.arm.LowBarStates;
import org.usfirst.frc.team4342.robot.components.Repository;

public class Shooter 
{
	public static final double JOYSTICK_DEADBAND = 0.05;

	private ShooterController shooter;
	private ArmController arm;

	private boolean ballPushStatus;
	private double autoMotorOutput;
	private int autoSetpoint;
	private LowBarStates state = LowBarStates.OVER;

	public Shooter(ShooterController shooter, ArmController arm)
	{
		this.shooter = shooter;
		this.arm = arm;
	}

	public void handleTeleop(int driverShootButton, int safetyButton, int fireButton, int smartDashboardButton, int armBrakeButton, int accumButton, int accumLiftButton)
	{
		shooter.checkUser(driverShootButton, safetyButton, fireButton, accumButton);
		arm.checkUser(smartDashboardButton, armBrakeButton, accumButton, accumLiftButton, safetyButton);
	}

	public void handleAuto()
	{
		arm.setSetpoint(autoSetpoint);
		shooter.setMotors(autoMotorOutput);
		shooter.setBallPusher(ballPushStatus);
	}

	public void setArmSetpoint(int setpoint)
	{
		this.autoSetpoint = setpoint;
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

	public void shootFromLowBar(int button)
	{
		if((Repository.Navx.getPitch() - 0.05) < 0.0 && Repository.DriveStick.getRawButton(button))
		{
			if(state == LowBarStates.OVER)
			{
				Repository.TankDrive.goStraight(-0.2, 0.0);

				if (Repository.Navx.getPitch() < 0.0)
					state = LowBarStates.BACKED_UP;
			}
			else if(state == LowBarStates.BACKED_UP)
			{
				Repository.DriveTrain.setCoastMode();

				if((Repository.Navx.getPitch() - 0.05) < 0.0)
					state = LowBarStates.STOP;
			}
			else if(state == LowBarStates.STOP)
			{
				Repository.DriveTrain.setBrakeMode();

				state = LowBarStates.TURN;
			}
			else if(state == LowBarStates.TURN)
			{
				Repository.TankDrive.goToAngle(54.14);

				if(Repository.Navx.getYaw() >= 53.5 || Repository.Navx.getYaw() <= 55.5) 
				{
					state = LowBarStates.AIM;
				}
			}
			else if(state == LowBarStates.AIM)
			{
				Repository.ArmController.setSetpoint(361.89); //25.42

				if(Repository.ArmController.isAtAutoSetpoint())
					state = LowBarStates.SHOOT;
			}
			else if(state == LowBarStates.SHOOT)
			{
				Repository.Shooter.shooter.setMotors(0.85);
				
				
				Repository.BallPusher.set(ballPushStatus);
				
				
				state = LowBarStates.OVER;
			}
		}
	}
}
