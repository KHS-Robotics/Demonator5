package org.usfirst.frc.team4342.api.logging;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.Map;

import org.usfirst.frc.team4342.robot.components.Repository;

import static org.usfirst.frc.team4342.robot.components.Repository.Navx;
import edu.wpi.first.wpilibj.CANTalon;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import ernie.logging.loggers.ILogger;
import ernie.logging.loggers.MultiLogger;

public class SmartDashboardUpdater 
{
	private SmartDashboardUpdater() {}

	private static boolean started;

	private static ArrayList<String> joystickKeys = new ArrayList<String>();
	private static ArrayList<String> talonKeys = new ArrayList<String>();
	private static ArrayList<String> limitSwitchKeys = new ArrayList<String>();
	private static ArrayList<String> encoderKeys = new ArrayList<String>();

	private static Map<String, Joystick> joysticks = new HashMap<String, Joystick>();
	private static Map<String, CANTalon> talons = new HashMap<String, CANTalon>();
	private static Map<String, DigitalInput> limitSwitches = new HashMap<String, DigitalInput>();
	private static Map<String, Encoder> encoders = new HashMap<String, Encoder>();

	/**
	 * Adds a joystick to put on the Smart Dashboard
	 * @param key the key to use when putting to the Smart Dashboard
	 * @param joystick the joystick to get data from
	 */
	public static void addJoystick(String key, Joystick joystick) 
	{
		joystickKeys.add(key);
		joysticks.put(key, joystick);
	}

	/**
	 * Adds a Jagaur to put on the Smart Dashboard
	 * @param key the key to use when putting to the Smart Dashboard
	 * @param jaguar the jagaur to get data from
	 */
	public static void addTalon(String key, CANTalon jaguar) 
	{
		talonKeys.add(key);
		talons.put(key, jaguar);
	}

	/**
	 * Adds a limit switch to put on the Smart Dashboard
	 * @param key the key to use when putting to the Smart Dashboard
	 * @param limitSwitch the limit switch to get data from
	 */
	public static void addDigitalInput(String key, DigitalInput limitSwitch) 
	{
		limitSwitchKeys.add(key);
		limitSwitches.put(key, limitSwitch);
	}

	public static void addEncoder(String key, Encoder encoder)
	{
		encoderKeys.add(key);
		encoders.put(key, encoder);
	}

	/**
	 * Starts updating the Smart Dashboard
	 */
	public static void startUpdating(ILogger log, RobotConsoleLogger consoleLog)
	{
		if(!started) 
		{
			new SmartDashboardUpdaterThread(log, consoleLog).start();
			started = true;
		}
	}

	/**
	 * The magic behind this class...
	 */
	private static class SmartDashboardUpdaterThread extends Thread implements Runnable 
	{	
		private boolean loggedJoysticks;
		private boolean loggedTalons;
		private boolean loggedDigitalInput;
		private boolean loggedNavx;
		private boolean loggedEncoders;

		private MultiLogger multiLog;

		public SmartDashboardUpdaterThread(ILogger log, RobotConsoleLogger consoleLog) 
		{
			this.multiLog = new MultiLogger(new ILogger[] { log, consoleLog });
		}

		/**
		 * Puts data to the Smart Dashboard
		 */
		@Override
		public void run() 
		{
			init();

			while(true) 
			{
				putJoystickData();
				putTalonData();
				putLimitSwitchData();
				putNavXData();
				putEncoderData();

				SmartDashboard.putString("Shooter-State", Repository.Shooter.getState().toString());

				try
				{
					Thread.sleep(50);
				} 
				catch(Exception ex)
				{
					multiLog.error(ExceptionInfo.getType(ex) + " in SDU", ex);
				}
			}
		}

		/**
		 * Initializes all the data to the Smart Dashboard
		 */
		private void init() 
		{
			try 
			{
				SmartDashboardUpdater.addJoystick("Joy-Drive", Repository.DriveStick);
				SmartDashboardUpdater.addJoystick("Joy-Control", Repository.ShooterStick);

				SmartDashboardUpdater.addTalon("FR", Repository.FrontRight);
				SmartDashboardUpdater.addTalon("FL", Repository.FrontLeft);
				SmartDashboardUpdater.addTalon("MR", Repository.MiddleRight);
				SmartDashboardUpdater.addTalon("ML", Repository.MiddleLeft);
				SmartDashboardUpdater.addTalon("RR", Repository.RearRight);
				SmartDashboardUpdater.addTalon("RL", Repository.RearLeft);
				SmartDashboardUpdater.addTalon("Arm", Repository.VerticalMotor);
				SmartDashboardUpdater.addTalon("RightShooter", Repository.RightShooter);
				SmartDashboardUpdater.addTalon("LeftShooter", Repository.LeftShooter);
				
				SmartDashboardUpdater.addEncoder("Arm-Enc", Repository.ArmEncoder);
			} 
			catch(Exception ex) 
			{
				multiLog.error("Failed to put data to SDBU", ex);
			}
		}

		/**
		 * Puts Joystick data to the Smart Dashboard
		 */
		private void putJoystickData() 
		{
			try 
			{
				for(String key : joystickKeys) 
				{
					Joystick joystick = joysticks.get(key);
					SmartDashboard.putNumber(key + "-X", joystick.getX());
					SmartDashboard.putNumber(key + "-Y", joystick.getY());
					SmartDashboard.putNumber(key + "-Z", joystick.getZ());
				}
			} 
			catch(Exception ex) 
			{
				if(!loggedJoysticks) 
				{
					multiLog.error("Error while putting Joystick data", ex);
					loggedJoysticks = true;
				}
			}
		}

		/**
		 * Puts Jagaur data to the Smart Dashboard
		 */
		private void putTalonData()
		{
			try 
			{
				for(String key : talonKeys)
				{
					CANTalon talon = talons.get(key);
					SmartDashboard.putNumber("Talon- " + key + "-Enc", talon.getPosition());
				}
			} 
			catch(Exception ex) 
			{
				if(!loggedTalons)
				{
					multiLog.error("Error while putting CANJaguar data", ex);
					loggedTalons = true;
				}
			}
		}

		/**
		 * Puts Limit Switch data to the Smart Dashboard
		 */
		private void putLimitSwitchData()
		{
			try 
			{
				for(String key : limitSwitchKeys) 
				{
					DigitalInput limitSwitch = limitSwitches.get(key);
					SmartDashboard.putBoolean(key, limitSwitch.get());
				}
			} 
			catch(Exception ex) 
			{
				if(!loggedDigitalInput) 
				{
					multiLog.error("Error while putting DigitalInput data", ex);
					loggedDigitalInput = true;
				}
			}
		}

		/**
		 * Puts NavX data to the Smart Dashboard
		 */
		private void putNavXData() 
		{
			try 
			{
				SmartDashboard.putNumber("NavX-Angle", Navx.getAngle());
				SmartDashboard.putNumber("NavX-Yaw", Navx.getYaw());
				SmartDashboard.putNumber("NavX-Pitch", Navx.getPitch());
				SmartDashboard.putNumber("NavX-Roll", Navx.getRoll());
				SmartDashboard.putNumber("NavX-Accel-X", Navx.getWorldLinearAccelX());
				SmartDashboard.putNumber("NavX-Accel-Y", Navx.getWorldLinearAccelY());
				SmartDashboard.putNumber("NavX-Accel-Z", Navx.getWorldLinearAccelZ());
				SmartDashboard.putNumber("NavX-Vel-X", Navx.getVelocityX());
				SmartDashboard.putNumber("NavX-Vel-Y", Navx.getVelocityY());
				SmartDashboard.putNumber("NavX-Vel-Z", Navx.getVelocityZ());
				SmartDashboard.putNumber("NavX-Disp-X", Navx.getDisplacementX());
				SmartDashboard.putNumber("NavX-Disp-Y", Navx.getDisplacementY());
				SmartDashboard.putNumber("NavX-Disp-Z", Navx.getDisplacementZ());
				SmartDashboard.putNumber("NavX-Mag-X", Navx.getRawMagX());
				SmartDashboard.putNumber("NavX-Mag-Y", Navx.getRawMagY());
				SmartDashboard.putNumber("NavX-Mag-Z", Navx.getRawMagZ());
				SmartDashboard.putBoolean("NavX-Connected", Navx.isConnected());
				SmartDashboard.putBoolean("NavX-Calibrating", Navx.isCalibrating());
			} 
			catch(Exception ex) 
			{
				if(!loggedNavx) 
				{
					multiLog.error("Error while putting Gyro data", ex);
					loggedNavx = true;
				}
			}
		}
		
		/**
		 * Puts Encoder data to the Smart Dashboard
		 */
		private void putEncoderData() 
		{
			try 
			{
				for(String key : encoderKeys) 
				{
					Encoder encoder = encoders.get(key);
					SmartDashboard.putNumber(key + "-Count", encoder.get());
				}
			} 
			catch(Exception ex) 
			{
				if(!loggedEncoders) 
				{
					multiLog.error("Error while putting Encoder data", ex);
					loggedEncoders = true;
				}
			}
		}
	}
}