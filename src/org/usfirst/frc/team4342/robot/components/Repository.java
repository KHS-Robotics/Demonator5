package org.usfirst.frc.team4342.robot.components;

import org.usfirst.frc.team4342.api.logging.ExceptionInfo;
import org.usfirst.frc.team4342.api.logging.PdpLogger;
import org.usfirst.frc.team4342.api.logging.RobotConsoleLogger;
import org.usfirst.frc.team4342.api.logging.RobotLogFactory;
import org.usfirst.frc.team4342.api.drive.DriveTrain;
import org.usfirst.frc.team4342.api.drive.TankDrive;
import org.usfirst.frc.team4342.api.shooter.Setpoint;
import org.usfirst.frc.team4342.api.shooter.SetpointMapWrapper;
import org.usfirst.frc.team4342.api.shooter.Shooter;

import edu.wpi.first.wpilibj.CANTalon;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.Encoder;
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
	public static Relay Accumulator;
	public static DriveTrain DriveTrain;
	public static DoubleSolenoid Shifter;
	public static Encoder DriveLeftEncoder, DriveRightEncoder;
	
	public static AHRS Navx;
	
	public static Relay CompressorRelay;
	
	public static CANTalon RightShooter, LeftShooter, VerticalMotor;
	public static Solenoid LoaderX, LoaderY;
	public static Encoder ArmEncoder;
	public static SetpointMapWrapper setpoints;
	
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
			initializeEncoders();
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
			Pdp = new PowerDistributionPanel();
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
			FrontRight = new CANTalon(13);
			FrontLeft = new CANTalon(2);
			MiddleRight = new CANTalon(14);
			MiddleLeft = new CANTalon(1);
			RearRight = new CANTalon(15);
			RearLeft = new CANTalon(0);
			
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
				//talon.changeControlMode(TalonControlMode.Speed);
				//talon.configEncoderCodesPerRev(512);
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
			RightShooter = new CANTalon(3);
			LeftShooter = new CANTalon(12);
			VerticalMotor = new CANTalon(4);
			
			Accumulator = new Relay(0, Relay.Direction.kForward);
			
//			RightShooter.changeControlMode(TalonControlMode.Speed);
//			LeftShooter.changeControlMode(TalonControlMode.Speed);
//			VerticalMotor.changeControlMode(TalonControlMode.Speed);
			
			RightShooter.enableBrakeMode(false);
			LeftShooter.enableBrakeMode(false);
			VerticalMotor.enableBrakeMode(true);
			
			RightShooter.enable();
			LeftShooter.enable();
			VerticalMotor.enable();
			
			ArmEncoder = new Encoder(4, 5);
			
			setpoints = new SetpointMapWrapper(
				new Setpoint[] {
					new Setpoint(1, 0)
				}
			);
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
			CompressorRelay = new Relay(1, Relay.Direction.kForward);
			Shifter = new DoubleSolenoid(0, 1);
			LoaderX = new Solenoid(7);
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
			TankDrive = new TankDrive(DriveStick, DriveTrain, Navx, Shifter, DriveLeftEncoder, DriveRightEncoder);
			Shooter = new Shooter(ShooterStick, Accumulator, RightShooter, LeftShooter, VerticalMotor, LoaderX, ArmEncoder, setpoints);
		}
		catch(Exception ex)
		{
			Logs.error("Failed to initialize main components of the robot (" + ExceptionInfo.getType(ex) + ")", ex);
		}
	}
	
	private static void initializeEncoders()
	{
		try
		{
			// TODO: DIO value conflicts? Getting AllocationExceptions...
			DriveRightEncoder = new Encoder(0, 1);
			DriveLeftEncoder = new Encoder(2, 3);
		}
		catch(Exception ex)
		{
			Logs.error("Failed to initialze encoders (" + ExceptionInfo.getType(ex) + ")", ex);
		}
	}
}
