package org.usfirst.frc.team4342.api.multithreading;

import org.usfirst.frc.team4342.api.logging.ExceptionInfo;
import org.usfirst.frc.team4342.api.pnuematics.Compressor;
import org.usfirst.frc.team4342.robot.components.Repository;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Relay;
import edu.wpi.first.wpilibj.Relay.Value;

public class CompressorComponent extends Component 
{
	private Relay relay;
	private DigitalInput pSwitch;
	
	public CompressorComponent(Compressor compressor)
	{
		this.relay = compressor.getRelay();
		this.pSwitch = compressor.getPressureSwitch();
	}

	@Override
	public void run() 
	{
		while(this.run)
		{
			try
			{
				final double velocity = Math.sqrt(Math.pow(Repository.Navx.getVelocityX(), 2) + Math.pow(Repository.Navx.getVelocityY(), 2));
				final double throttle = Math.sqrt(Math.pow(Repository.DriveStick.getX(), 2) + Math.pow(Repository.DriveStick.getY(), 2));
				final double ratio = (velocity/throttle);
				final double PushValue = 20.0;
				
				if(pSwitch.get() || (ratio < PushValue))
				{
					relay.set(Value.kOff);
				}
				else
				{
					relay.set(Value.kForward);
				}
				
				Thread.sleep(100);
			}
			catch(Exception ex)
			{
				Repository.Logs.error(
					"Unexpected error with compressor (" + ExceptionInfo.getType(ex) + ")", 
					ex
				);
				
				this.run = false;
				break;
			}
		}
	}
}
