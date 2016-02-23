package org.usfirst.frc.team4342.api.multithreading;

import org.usfirst.frc.team4342.api.drive.TankDrive;
import org.usfirst.frc.team4342.robot.components.Repository;

import edu.wpi.first.wpilibj.DriverStation;

public class TankDriveComponent extends Component
{
	private TankDrive td;
	
	public TankDriveComponent(TankDrive td)
	{
		this.td = td;

	}
	
	@Override
	public void stopAll()
	{
		td.stopAll();
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
					td.drive(5, 10, -1, 5, 6);
				}
				else if(DriverStation.getInstance().isEnabled() && DriverStation.getInstance().isAutonomous())
				{
					
				}
				
				Thread.sleep(Component.SLEEP_MILLIS);
			}
			catch(Exception ex)
			{
				Repository.Logs.error(
					"Unexpected error in DriveTrain thread", 
					ex
				);
				
				this.run = false;
				break;
			}
		}
	}
}
