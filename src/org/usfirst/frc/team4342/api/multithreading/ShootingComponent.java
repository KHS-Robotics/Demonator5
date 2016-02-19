package org.usfirst.frc.team4342.api.multithreading; 

import org.usfirst.frc.team4342.api.shooter.Shooter;
import org.usfirst.frc.team4342.robot.components.Repository;
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
					shooter.handleTeleop(5, 10, 4, 6, 3);
				}
				else if(DriverStation.getInstance().isEnabled() && DriverStation.getInstance().isOperatorControl())
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
				
				this.run = false;
				break;
			}
		}
	}
}
