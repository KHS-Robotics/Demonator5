package org.usfirst.frc.team4342.api.multithreading; 

import org.usfirst.frc.team4342.api.logging.ExceptionInfo;
import org.usfirst.frc.team4342.api.shooter.Shooter;
import org.usfirst.frc.team4342.api.shooter.ShooterState;
import org.usfirst.frc.team4342.robot.components.Repository;

import edu.wpi.first.wpilibj.CANTalon;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.Solenoid;

public class ShootingComponent extends Component
{
	private Joystick j;
	private CANTalon accumulator, rightMotor, leftMotor;
	private Solenoid loaderX, loaderY;
	
	public ShootingComponent(Shooter shooter)
	{
		this.j = shooter.getJoystick();
		this.accumulator = shooter.getAccumulator();
		this.rightMotor = shooter.getRightMotor();
		this.leftMotor = shooter.getLeftMotor();
		this.loaderX = shooter.getLoaderX();
		this.loaderY = shooter.getLoaderY();
	}
	
	@Override
	public void run()
	{
		ShooterState state = loaderX.get() ? ShooterState.FIRED : ShooterState.LOADED;
		
		while(this.run)
		{
			try
			{
				if(DriverStation.getInstance().isEnabled() && DriverStation.getInstance().isOperatorControl())
				{
					if (state == ShooterState.LOADED)
					{
						if (j.getRawButton(0))
						{
							loaderY.set(true);
							rightMotor.set(1);
							leftMotor.set(1);
							
							if (j.getRawButton(1) && (rightMotor.getEncVelocity() > 30 && leftMotor.getEncVelocity() > 30))
								state = ShooterState.FIRING;
						}
						else
						{
							loaderY.set(false);
							rightMotor.set(0);
							leftMotor.set(0);
						}
					}
					else if (state == ShooterState.FIRING)
					{
						loaderX.set(true);
						Thread.sleep(250);
						rightMotor.set(0);
						leftMotor.set(0);
						state = ShooterState.FIRED;
					}
					else if (state == ShooterState.FIRED)
					{
						if (!j.getRawButton(0))
						{
							state = ShooterState.RELOADING;
						}
					}
					else if (state == ShooterState.RELOADING)
					{
						loaderY.set(false);
						loaderX.set(false);
						state = ShooterState.LOADED;
					}
					
					if(j.getRawButton(3))
						accumulator.set(-1);
					else
						accumulator.set(0);
				}
				
				Thread.sleep(50);
			}
			catch(Exception ex)
			{
				Repository.Logs.error(
					"Unexpected error in shooter (" + ExceptionInfo.getType(ex) + ")", 
					ex
				);
				
				this.run = false;
				break;
			}
		}
	}
}
