package org.usfirst.frc.team4342.robot.multithreading; 

import org.usfirst.frc.team4342.api.shooter.Shooter;
import org.usfirst.frc.team4342.robot.Repository;

import edu.wpi.first.wpilibj.DriverStation;

public class ShootingComponent extends Component
{
	private Shooter shooter;
	
	public ShootingComponent(Shooter shooter)
	{
		this.shooter = shooter;
	}
	
	@Override
	public void stopAll()
	{
		shooter.stopAll();
	}
	
	@Override
	public void run()
	{
		while(this.run)
		{
			try
			{
				if(DriverStation.getInstance().isEnabled() && DriverStation.getInstance().isOperatorControl())
				{
					shooter.handleTeleop(1, 5, 10, 3, 2, 6, 2);
				}
				else if(DriverStation.getInstance().isEnabled() && DriverStation.getInstance().isAutonomous())
				{
					shooter.handleAuto();
				}
				
				Thread.sleep(Component.SLEEP_MILLIS);
			}
			catch(Exception ex)
			{
				Repository.Logs.error(
					"Unexpected error in shooter thread", 
					ex
				);
				
				ComponentRunner.stopAutomaticMode(this);
				break;
			}
		}
	}
}
