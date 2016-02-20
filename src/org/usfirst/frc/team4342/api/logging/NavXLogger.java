package org.usfirst.frc.team4342.api.logging;

import java.io.BufferedWriter;
import java.io.FileWriter;
import java.io.IOException;
import java.text.SimpleDateFormat;
import java.util.Date;
import java.util.TimeZone;

import org.usfirst.frc.team4342.api.drive.DriveTrain;
import org.usfirst.frc.team4342.robot.components.Repository;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.CANTalon;

public class NavXLogger 
{
	private NavXLogger() {}
	
	private static final String RETURN_FEED = System.getProperty("line.separator");
	private static final char COMMA = ',';
	
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
					
					Thread.sleep(20);
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
				
				writer.write("Yaw" + COMMA);
				writer.write("Roll" + COMMA);
				writer.write("Pitch" + COMMA);
				writer.write("Acceleration-X" + COMMA);
				writer.write("Acceleration-Y" + COMMA);
				writer.write("Acceleration-Z" + COMMA);
				writer.write("Velocity-X" + COMMA);
				writer.write("Velocity-Y" + COMMA);
				writer.write("Velocity-Z" + COMMA);
				writer.write("Displacement-X" + COMMA);
				writer.write("Displacement-Y" + COMMA);
				writer.write("Displacement-Z" + COMMA);
				writer.write("Rate" + COMMA);
				writer.write("Fused Heading" + COMMA);
				
				writer.write("FrontRight-Get" + COMMA);
				writer.write("FrontRight-Setpoint" + COMMA);
				writer.write("FrontRight-BusVoltage" + COMMA);
				writer.write("FrontRight-OutputCurrent" + COMMA);
				writer.write("FrontRight-OutputVoltage" + COMMA);
				
				writer.write("FrontLeft-Get" + COMMA);
				writer.write("FrontLeft-Setpoint" + COMMA);
				writer.write("FrontLeft-BusVoltage" + COMMA);
				writer.write("FrontLeft-OutputCurrent" + COMMA);
				writer.write("FrontLeft-OutputVoltage" + COMMA);
				
				writer.write("MiddleRight-Get" + COMMA);
				writer.write("MiddleRight-Setpoint" + COMMA);
				writer.write("MiddleRight-BusVoltage" + COMMA);
				writer.write("MiddleRight-OutputCurrent" + COMMA);
				writer.write("MiddleRight-OutputVoltage" + COMMA);
				
				writer.write("MiddleLeft-Get" + COMMA);
				writer.write("MiddleLeft-Setpoint" + COMMA);
				writer.write("MiddleLeft-BusVoltage" + COMMA);
				writer.write("MiddleLeft-OutputCurrent" + COMMA);
				writer.write("MiddleLeft-OutputVoltage" + COMMA);
				
				writer.write("RearRight-Get" + COMMA);
				writer.write("RearRight-Setpoint" + COMMA);
				writer.write("RearRight-BusVoltage" + COMMA);
				writer.write("RearRight-OutputCurrent" + COMMA);
				writer.write("RearRight-OutputVoltage" + COMMA);
				
				writer.write("RearLeft-Get" + COMMA);
				writer.write("RearLeft-Setpoint" + COMMA);
				writer.write("RearLeft-BusVoltage" + COMMA);
				writer.write("RearLeft-OutputCurrent" + COMMA);
				writer.write("RearLeft-OutputVoltage" + COMMA);
				
				writer.write("Time" + RETURN_FEED);
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
			writer.write("" + navx.getYaw() + COMMA);
			writer.write("" + navx.getRoll() + COMMA);
			writer.write("" + navx.getPitch() + COMMA);
			writer.write("" + navx.getWorldLinearAccelX() + COMMA);
			writer.write("" + navx.getWorldLinearAccelY() + COMMA);
			writer.write("" + navx.getWorldLinearAccelZ() + COMMA);
			writer.write("" + navx.getVelocityX() + COMMA);
			writer.write("" + navx.getVelocityY() + COMMA);
			writer.write("" + navx.getVelocityZ() + COMMA);
			writer.write("" + navx.getDisplacementX() + COMMA);
			writer.write("" + navx.getDisplacementY() + COMMA);
			writer.write("" + navx.getDisplacementZ() + COMMA);
			writer.write("" + navx.getRate() + COMMA);
			writer.write("" + navx.getFusedHeading() + COMMA);
		}
		
		private void logDriveTrainData() throws IOException
		{
			for(CANTalon talon : drive.getDriveTrain())
			{
				writer.write("" + talon.get() + COMMA);
				writer.write("" + drive.getPIDController().getSetpoint() + COMMA);
				writer.write("" + talon.getBusVoltage() + COMMA);
				writer.write("" + talon.getOutputCurrent() + COMMA);
				writer.write("" + talon.getOutputVoltage() + COMMA);
			}
			
			writer.write(getCurrentTime());
			writer.write(RETURN_FEED);
		}
		
		@SuppressWarnings("deprecation")
		private String getCurrentTime()
		{
			long ts = System.currentTimeMillis();
	        Date localTime = new Date(ts);
	        String format = "yyyy/MM/dd HH:mm:ss";
	        SimpleDateFormat sdf = new SimpleDateFormat(format);

	        // Convert Local Time to UTC
	        sdf.setTimeZone(TimeZone.getTimeZone("UTC"));
	        Date gmtTime = new Date(sdf.format(localTime));

	        // Convert UTC to Local Time
	        Date fromGmt = new Date(gmtTime.getTime() + TimeZone.getDefault().getOffset(localTime.getTime()));
	       
	        // parse out to hh:mm:ss
	        String date = fromGmt.toString();
	        date = date.substring(date.indexOf(":")-2, 20);
	       
	        return date;
		}
	}
}
