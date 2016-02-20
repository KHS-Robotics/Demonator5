package org.usfirst.frc.team4342.api.logging;

import java.io.BufferedWriter;
import java.io.FileWriter;
import java.io.IOException;

import org.usfirst.frc.team4342.api.drive.DriveTrain;
import org.usfirst.frc.team4342.robot.components.Repository;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.CANTalon;

public class NavXLogger 
{
	private NavXLogger() {}
	
	private static final String RETURN_FEED = System.getProperty("line.separator");
	
	private static boolean isLogging;
	
	public static void startLogging(AHRS navx, DriveTrain drive)
	{
		if(isLogging)
			return;
		
		new NavXLoggingThread(navx, drive).start();
		isLogging = true;
	}
	
	public static void stopLogging()
	{
		isLogging = false;
	}
	
	private static class NavXLoggingThread extends Thread implements Runnable
	{
		private AHRS navx;
		private DriveTrain drive;
		private static BufferedWriter writer;
		
		public NavXLoggingThread(AHRS navx, DriveTrain drive)
		{
			this.navx = navx;
			this.drive = drive;
		}
		
		@Override
		public void run()
		{
			init();
			
			while(isLogging)
			{
				try
				{
					logNavXData();
					logDriveTrainData();
				}
				catch(Exception ex)
				{
					Repository.Log.error("Failed to log NavX data!", ex);
					Repository.ConsoleLog.warning("Failed to log NavX data!");
					isLogging = false;
					break;
				}
			}
		}
		
		private void init()
		{
			try
			{
				writer = new BufferedWriter(new FileWriter(FileHelper.getValidNavXLogFile()));
				
				writer.write("Yaw" + RETURN_FEED);
				writer.write("Roll" + RETURN_FEED);
				writer.write("Pitch" + RETURN_FEED);
				writer.write("Acceleration-X" + RETURN_FEED);
				writer.write("Acceleration-Y" + RETURN_FEED);
				writer.write("Acceleration-Z" + RETURN_FEED);
				writer.write("Velocity-X" + RETURN_FEED);
				writer.write("Velocity-Y" + RETURN_FEED);
				writer.write("Velocity-Z" + RETURN_FEED);
				writer.write("Displacement-X" + RETURN_FEED);
				writer.write("Displacement-Y" + RETURN_FEED);
				writer.write("Displacement-Z" + RETURN_FEED);
				writer.write("Rate" + RETURN_FEED);
				writer.write("Fused Heading" + RETURN_FEED);
				
				writer.write("FrontRight-Get" + RETURN_FEED);
				writer.write("FrontRight-Setpoint" + RETURN_FEED);
				writer.write("FrontRight-BusVoltage" + RETURN_FEED);
				writer.write("FrontRight-OutputCurrent" + RETURN_FEED);
				writer.write("FrontRight-OutputVoltage" + RETURN_FEED);
				
				writer.write("FrontLeft-Get" + RETURN_FEED);
				writer.write("FrontLeft-Setpoint" + RETURN_FEED);
				writer.write("FrontLeft-BusVoltage" + RETURN_FEED);
				writer.write("FrontLeft-OutputCurrent" + RETURN_FEED);
				writer.write("FrontLeft-OutputVoltage" + RETURN_FEED);
				
				writer.write("MiddleRight-Get" + RETURN_FEED);
				writer.write("MiddleRight-Setpoint" + RETURN_FEED);
				writer.write("MiddleRight-BusVoltage" + RETURN_FEED);
				writer.write("MiddleRight-OutputCurrent" + RETURN_FEED);
				writer.write("MiddleRight-OutputVoltage" + RETURN_FEED);
				
				writer.write("MiddleLeft-Get" + RETURN_FEED);
				writer.write("MiddleLeft-Setpoint" + RETURN_FEED);
				writer.write("MiddleLeft-BusVoltage" + RETURN_FEED);
				writer.write("MiddleLeft-OutputCurrent" + RETURN_FEED);
				writer.write("MiddleLeft-OutputVoltage" + RETURN_FEED);
				
				writer.write("RearRight-Get" + RETURN_FEED);
				writer.write("RearRight-Setpoint" + RETURN_FEED);
				writer.write("RearRight-BusVoltage" + RETURN_FEED);
				writer.write("RearRight-OutputCurrent" + RETURN_FEED);
				writer.write("RearRight-OutputVoltage" + RETURN_FEED);
				
				writer.write("RearLeft-Get" + RETURN_FEED);
				writer.write("RearLeft-Setpoint" + RETURN_FEED);
				writer.write("RearLeft-BusVoltage" + RETURN_FEED);
				writer.write("RearLeft-OutputCurrent" + RETURN_FEED);
				writer.write("RearLeft-OutputVoltage" + RETURN_FEED);
			}
			catch(Exception ex)
			{
				Repository.Log.error("Failed to initialize NavX logger!", ex);
				Repository.ConsoleLog.warning("Failed to log NavX data!");
				isLogging = false;
			}
		}
		
		private void logNavXData() throws IOException
		{
			writer.write("" + navx.getYaw());
			writer.write("" + navx.getRoll());
			writer.write("" + navx.getPitch());
			writer.write("" + navx.getWorldLinearAccelX());
			writer.write("" + navx.getWorldLinearAccelY());
			writer.write("" + navx.getWorldLinearAccelZ());
			writer.write("" + navx.getVelocityX());
			writer.write("" + navx.getVelocityY());
			writer.write("" + navx.getVelocityZ());
			writer.write("" + navx.getDisplacementX());
			writer.write("" + navx.getDisplacementY());
			writer.write("" + navx.getDisplacementZ());
			writer.write("" + navx.getRate());
			writer.write("" + navx.getFusedHeading());
		}
		
		private void logDriveTrainData() throws IOException
		{
			for(CANTalon talon : drive.getDriveTrain())
			{
				writer.write("" + talon.get());
				writer.write("" + drive.getPIDController().getSetpoint());
				writer.write("" + talon.getBusVoltage());
				writer.write("" + talon.getOutputCurrent());
				writer.write("" + talon.getOutputVoltage());
			}
		}
	}
}
