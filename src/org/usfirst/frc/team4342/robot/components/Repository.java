package org.usfirst.frc.team4342.robot.components;

import org.usfirst.frc.team4342.api.logging.ExceptionInfo;
import org.usfirst.frc.team4342.api.logging.PdpLogger;
import org.usfirst.frc.team4342.api.logging.RobotConsoleLogger;
import org.usfirst.frc.team4342.api.logging.RobotLogFactory;
import org.usfirst.frc.team4342.api.drive.DriveTrain;
import org.usfirst.frc.team4342.api.pnuematics.Compressor;
import org.usfirst.frc.team4342.api.drive.TankDrive;
import org.usfirst.frc.team4342.api.shooter.Shooter;

import edu.wpi.first.wpilibj.CANTalon;
import edu.wpi.first.wpilibj.CANTalon.TalonControlMode;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.PowerDistributionPanel;
import edu.wpi.first.wpilibj.Relay;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.Solenoid;

import com.kauailabs.navx.frc.AHRS;

import ernie.logging.loggers.ILogger;
import ernie.logging.loggers.MultiLogger;

/**
 * Poor man's IoC. This code is the first thing that runs in Robot.robotInit() to initialize
 * all of the objects the robot uses to make them all easily accessible. Of course though Java
 * is dumb and doesn't have properties (getters and setters in one line) so we are lazy and just
 * made the instance variables public. Yay Java and encapsulation! #DealWithIt
 */
public class Repository 
{
	private static boolean initialized;
	
	public static ILogger Log;
	public static RobotConsoleLogger ConsoleLog;
	public static MultiLogger Logs;
	
	public static Joystick DriveStick, ShooterStick, SwitchBox;
	
	public static PowerDistributionPanel Pdp;
	
	public static CANTalon FrontRight, FrontLeft, MiddleRight,
							MiddleLeft, RearRight, RearLeft;
	public static CANTalon RightShooter, LeftShooter, Accumulator, VerticalMotor;
	public static DriveTrain DriveTrain;
	
	public static AHRS Navx;
	
	public static DigitalInput PressureSwitch;
	public static Relay CompressorRelay;
	public static DoubleSolenoid Shifter;
	public static Solenoid LoaderX, LoaderY;
	
	public static Compressor Compressor;
	public static TankDrive TankDrive;
	public static Shooter Shooter;
	
	public static void initializeAll()
	{
		if(!initialized)
		{
			initializeLogs();
			initializeJoysticks();
			initializeDrive();
			initializeShooter();
			initializePneumatics();
			initializeComponents();
		}
		
		initialized = true;
	}
	
	private static void initializeLogs()
	{
		try
		{
			ConsoleLog = RobotLogFactory.createRobotConsoleLogger();
			Log = RobotLogFactory.createAsyncLogger();
			Logs = new MultiLogger(new ILogger[] { ConsoleLog, Log });
		}
		catch(Exception ex)
		{
			ConsoleLog.warning("Failed to initialize log due to a " + ExceptionInfo.getType(ex));
			Logs = new MultiLogger(new ILogger[] { ConsoleLog });
		}
		
		try
		{
			PdpLogger.start(Pdp, Log, ConsoleLog);
		}
		catch(Exception ex)
		{
			Logs.warning("Failed to start PDP Logger due to a " + ExceptionInfo.getType(ex));
		}
	}
	
	private static void initializeJoysticks()
	{
		try
		{
			DriveStick = new Joystick(0);
			ShooterStick = new Joystick(1);
			SwitchBox = new Joystick(2);
		}
		catch(Exception ex)
		{
			Logs.error("Failed to initialize joysticks due to a " + ExceptionInfo.getType(ex), ex);
		}
	}
	
	private static void initializeDrive()
	{
		try
		{
			FrontRight = new CANTalon(0);
			FrontLeft = new CANTalon(15);
			MiddleRight = new CANTalon(1);
			MiddleLeft = new CANTalon(14);
			RearRight = new CANTalon(2);
			RearLeft = new CANTalon(13);
			
			DriveTrain = new DriveTrain(
				DriveStick,
				FrontRight,
				FrontLeft,
				MiddleRight,
				MiddleLeft,
				RearRight,
				RearLeft
			);
			
			for(CANTalon talon : DriveTrain.getDriveTrain())
			{
				talon.changeControlMode(TalonControlMode.Speed);
				talon.configEncoderCodesPerRev(512);
				talon.enableBrakeMode(true);
				talon.enable();
			}
			
			Navx = new AHRS(SPI.Port.kMXP, (byte)50);
		}
		catch(Exception ex)
		{
			Logs.error("Failed to initialize drive due to a " + ExceptionInfo.getType(ex), ex);
		}
	}
	
	private static void initializeShooter()
	{
		try
		{
			RightShooter = new CANTalon(-1);
			LeftShooter = new CANTalon(-2);
			Accumulator = new CANTalon(-3);
			
			RightShooter.changeControlMode(TalonControlMode.Speed);
			LeftShooter.changeControlMode(TalonControlMode.Speed);
			Accumulator.changeControlMode(TalonControlMode.Speed);
			
			RightShooter.enableBrakeMode(false);
			LeftShooter.enableBrakeMode(false);
			Accumulator.enableBrakeMode(false);
			
			RightShooter.enable();
			LeftShooter.enable();
			Accumulator.enable();
		}
		catch(Exception ex)
		{
			Logs.error("Failed to initialize shooter due to a " + ExceptionInfo.getType(ex), ex);
		}
	}
	
	private static void initializePneumatics()
	{
		try
		{
			PressureSwitch = new DigitalInput(1);
			CompressorRelay = new Relay(1, Relay.Direction.kForward);
			Shifter = new DoubleSolenoid(1, 2);
			LoaderX = new Solenoid(3);
			LoaderY = new Solenoid(4);
		}
		catch(Exception ex)
		{
			Logs.error("Failed to initialize pneumatics due to a " + ExceptionInfo.getType(ex), ex);
		}
	}
	
	private static void initializeComponents()
	{
		try
		{
			Compressor = new Compressor(CompressorRelay, PressureSwitch);
			TankDrive = new TankDrive(DriveStick, DriveTrain, Navx, Shifter);
			Shooter = new Shooter(ShooterStick, Accumulator, RightShooter, LeftShooter, LoaderX, LoaderY);
		}
		catch(Exception ex)
		{
			Logs.error("Failed to initialize main components of the robot (" + ExceptionInfo.getType(ex) + ")", ex);
		}
	}
}
