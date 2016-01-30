package org.usfirst.frc.team4342.api.multithreading;

import org.usfirst.frc.team4342.api.logging.ExceptionInfo;
import org.usfirst.frc.team4342.api.pnuematics.Compressor;
import org.usfirst.frc.team4342.robot.components.Repository;

public class CompressorComponent extends Component 
{
	private Compressor compressor;
	
	public CompressorComponent(Compressor compressor)
	{
		this.compressor = compressor;
	}

	@Override
	public void run() 
	{
		while(this.run)
		{
			try
			{
				compressor.handle();
				
				Thread.sleep(Component.SLEEP_MILLIS);
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
