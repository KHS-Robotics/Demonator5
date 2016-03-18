package org.usfirst.frc.team4342.api.logging;

import java.util.HashMap;
import java.util.Map;
import java.util.Map.Entry;

import org.usfirst.frc.team4342.api.autonomous.AutoRoutinesRunner;
import org.usfirst.frc.team4342.api.drive.DefenseState;
import org.usfirst.frc.team4342.robot.components.Repository;

import static org.usfirst.frc.team4342.robot.components.Repository.Navx;
import edu.wpi.first.wpilibj.CANTalon;
import edu.wpi.first.wpilibj.Counter;
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
	
	private static Map<String, Joystick> joysticks = new HashMap<String, Joystick>();
	private static Map<String, CANTalon> talons = new HashMap<String, CANTalon>();
	private static Map<String, DigitalInput> limitSwitches = new HashMap<String, DigitalInput>();
	private static Map<String, Encoder> encoders = new HashMap<String, Encoder>();
	private static Map<String, Counter> counters = new HashMap<String, Counter>();
	private static Map<String, DefenseState> defenseStates = new HashMap<String, DefenseState>();

	/**
	 * Adds a joystick to put on the Smart Dashboard
	 * @param key the key to use when putting to the Smart Dashboard
	 * @param joystick the joystick to get data from
	 */
	public static void addJoystick(String key, Joystick joystick) 
	{
		joysticks.put(key, joystick);
	}

	/**
	 * Adds a talon to put on the Smart Dashboard
	 * @param key the key to use when putting to the Smart Dashboard
	 * @param talon the talon to get data from
	 */
	public static void addTalon(String key, CANTalon talon) 
	{
		talons.put(key, talon);
	}

	/**
	 * Adds a limit switch to put on the Smart Dashboard
	 * @param key the key to use when putting to the Smart Dashboard
	 * @param limitSwitch the limit switch to get data from
	 */
	public static void addDigitalInput(String key, DigitalInput limitSwitch) 
	{
		limitSwitches.put(key, limitSwitch);
	}

	public static void addEncoder(String key, Encoder encoder)
	{
		encoders.put(key, encoder);
	}
	
	public static void addCounter(String key, Counter counter)
	{
		counters.put(key, counter);
	}
	
	public static void addDefenseState(String key, DefenseState state)
	{
		defenseStates.put(key, state);
	}

	/**
	 * Starts updating the Smart Dashboard
	 */
	public static void startUpdating(ILogger log, RobotConsoleLogger consoleLog)
	{
		if(!started) 
		{
			SmartDashboard.putBoolean("SDU-Errored", false);
			new SmartDashboardUpdaterThread(log, consoleLog).start();
			started = true;
		}
	}

	/**
	 * The magic behind this class...
	 */
	private static class SmartDashboardUpdaterThread extends Thread implements Runnable 
	{	
		private boolean errored;
		private boolean loggedJoysticks;
		private boolean loggedTalons;
		private boolean loggedDigitalInput;
		private boolean loggedNavx;
		private boolean loggedEncoders;
		private boolean loggedCounters;
		private boolean loggedDefenseStates;

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
			try
			{
				while(true)
				{
					putJoystickData();
					putTalonData();
					putLimitSwitchData();
					putNavXData();
					putEncoderData();
					putCounterData();
					putDefenseStates();
					putSmartDashobardData();
					
					SmartDashboard.putString("Shooter-State", Repository.ShooterController.getState().toString());
	
					Thread.sleep(50);
				}
			}
			catch(Exception ex)
			{
				if(!errored)
				{
					Repository.Logs.error("Failed to put data to SmartDashboard", ex);
					SmartDashboard.putBoolean("SDU-Errored", true);
				}
				
				errored = true;
			}
		}
		
		private static void putSmartDashobardData()
		{
			SmartDashboard.putNumber("AutoStep", AutoRoutinesRunner.getCurrentStep());
			SmartDashboard.putBoolean("AutoFinished", AutoRoutinesRunner.isFinished());
			SmartDashboard.putBoolean("AutoErrored", AutoRoutinesRunner.hasErrored());
			SmartDashboard.putNumber("AutoTimerSeconds", Repository.Timer.get());
		}


		/**
		 * Puts Joystick data to the Smart Dashboard
		 */
		private void putJoystickData() 
		{
			try 
			{
				for(Entry<String, Joystick> entry : joysticks.entrySet())
				{
					String key = entry.getKey();
					Joystick joystick = entry.getValue();
					
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
		 * Puts Talon data to the Smart Dashboard
		 */
		private void putTalonData()
		{
			try 
			{
				for(Entry<String, CANTalon> entry : talons.entrySet())
				{
					String key = entry.getKey();
					CANTalon talon = entry.getValue();
					
					SmartDashboard.putNumber("Talon-" + key + "-Get", talon.get());
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
				for(Entry<String, DigitalInput> entry : limitSwitches.entrySet())
				{
					String key = entry.getKey();
					DigitalInput limitSwitch = entry.getValue();
					
					SmartDashboard.putBoolean(key + "-Get", limitSwitch.get());
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
				SmartDashboard.putBoolean("NavX-Connected", Navx.isConnected());
				SmartDashboard.putBoolean("NavX-Calibrating", Navx.isCalibrating());
			} 
			catch(Exception ex) 
			{
				if(!loggedNavx) 
				{
					multiLog.error("Error while putting NavX data", ex);
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
				for(Entry<String, Encoder> entry : encoders.entrySet())
				{
					String key = entry.getKey();
					Encoder encoder = entry.getValue();
					
					SmartDashboard.putNumber(key + "-Dist", encoder.getDistance());
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
		
		/**
		 * Puts Counter data to the Smart Dashboard
		 */
		private void putCounterData()
		{
			try
			{
				for(Entry<String, Counter> entry : counters.entrySet())
				{
					String key = entry.getKey();
					Counter counter = entry.getValue();
					
					SmartDashboard.putNumber(key + "-Rate", counter.getRate());
					SmartDashboard.putNumber(key + "-Period", counter.getPeriod());
					SmartDashboard.putNumber(key + "-Get", counter.get());
					SmartDashboard.putNumber(key + "-Dist", counter.getDistance());
				}
			}
			catch(Exception ex)
			{
				if(!loggedCounters)
				{
					multiLog.error("Error while putting Counter data", ex);
					loggedCounters = true;
				}
			}
		}
		
		/**
		 * Puts defense data to the Smart Dashboard
		 */
		public void putDefenseStates()
		{
			try
			{
				for(Entry<String, DefenseState> entry : defenseStates.entrySet())
				{
					String key = entry.getKey();
					DefenseState state = entry.getValue();
					
					SmartDashboard.putString(key, state.toString());
				}
			}
			catch(Exception ex)
			{
				if(!loggedDefenseStates)
				{
					multiLog.error("Error while putting Defense State data", ex);
					loggedDefenseStates = true;
				}
			}
		}
	}
}