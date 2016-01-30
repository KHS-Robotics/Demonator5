package org.usfirst.frc.team4342.api.multithreading;

import org.usfirst.frc.team4342.api.drive.TankDrive;
import org.usfirst.frc.team4342.api.logging.ExceptionInfo;
import org.usfirst.frc.team4342.robot.components.Repository;

import edu.wpi.first.wpilibj.DriverStation;

public class TankDriveComponent extends Component
{
	private TankDrive td;
	private int shiftButton;
	
	public TankDriveComponent(TankDrive td, int shiftButton)
	{
		this.td = td;
		this.shiftButton = shiftButton;
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
					td.drive(shiftButton);
				}
				
				Thread.sleep(Component.SLEEP_MILLIS);
			}
			catch(Exception ex)
			{
				Repository.Logs.error(
					"Unexpected error in DriveTrain (" + ExceptionInfo.getType(ex) + ")", 
					ex
				);
				
				this.run = false;
				break;
			}
		}
	}
}
