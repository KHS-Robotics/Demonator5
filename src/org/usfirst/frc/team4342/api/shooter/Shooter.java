 package org.usfirst.frc.team4342.api.shooter;

import org.usfirst.frc.team4342.api.logging.ExceptionInfo;
import org.usfirst.frc.team4342.robot.components.Repository;

import edu.wpi.first.wpilibj.CANTalon;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.Solenoid;

public class Shooter 
{
	private static boolean run;
	
	public static void startAutomaticMode(Joystick j, CANTalon accumulator, CANTalon rightMotor, 
										  CANTalon leftMotor, Solenoid loaderX, Solenoid loaderY, 
										  int extendButton)
	{
		if(run)
			return;
		
		run = true;
		
		Thread t = new Thread(new Runnable()
		{
			ShooterState state = loaderX.get() ? ShooterState.FIRED : ShooterState.LOADED;
			
			@Override
			public void run()
			{
				while(run)
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
						
						run = false;
						break;
					}
				}
			}
		});
		
		t.start();
	}
	
	public static void stopAutomaticMode()
	{
		run = false;
	}
}
