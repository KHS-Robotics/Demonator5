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
import edu.wpi.first.wpilibj.DigitalInput;
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
	
	public static DriveTrain DriveTrain;
	public static DoubleSolenoid Shifter;
	public static Encoder LeftDriveEncoder, RightDriveEncoder;
	
	public static AHRS Navx;
	
	public static CANTalon RightShooter, LeftShooter, ArmMotor, Accumulator;
	public static Solenoid BallPusher;
	public static Encoder ArmEncoder;
	public static DigitalInput BallSensor;
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
			initializeEncoders();
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
			Pdp = new PowerDistributionPanel();
			PdpLogger.start(Pdp, Log, ConsoleLog);
		}
		catch(Exception ex)
		{
			Logs.warning("Failed to start PDP Logger");
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
			Logs.error("Failed to initialize joysticks", ex);
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
			Logs.error("Failed to initialize drive", ex);
		}
	}
	
	private static void initializeShooter()
	{
		try
		{
			RightShooter = new CANTalon(3);
			LeftShooter = new CANTalon(11);
			ArmMotor = new CANTalon(4);
			Accumulator = new CANTalon(12);
			
//			RightShooter.changeControlMode(TalonControlMode.Speed);
//			LeftShooter.changeControlMode(TalonControlMode.Speed);
//			VerticalMotor.changeControlMode(TalonControlMode.Speed);
			
			RightShooter.enableBrakeMode(false);
			LeftShooter.enableBrakeMode(false);
			ArmMotor.enableBrakeMode(true);
			Accumulator.enableBrakeMode(true);
			
			RightShooter.enable();
			LeftShooter.enable();
			ArmMotor.enable();
			Accumulator.enable();
			
			ArmEncoder = new Encoder(4, 5);
			
			setpoints = new SetpointMapWrapper(
				new Setpoint[] {
					new Setpoint(1, 0)
				}
			);
			
			BallSensor = new DigitalInput(6);
		}
		catch(Exception ex)
		{
			Logs.error("Failed to initialize shooter", ex);
		}
	}
	
	private static void initializePneumatics()
	{
		try
		{
			Shifter = new DoubleSolenoid(0, 1);
			BallPusher = new Solenoid(3);
		}
		catch(Exception ex)
		{
			Logs.error("Failed to initialize pneumatics", ex);
		}
	}
	
	private static void initializeComponents()
	{
		try
		{
			TankDrive = new TankDrive(DriveStick, DriveTrain, Navx, Shifter, LeftDriveEncoder, RightDriveEncoder);
			Shooter = new Shooter(
				ShooterStick, 
				Accumulator, 
				RightShooter,
				LeftShooter, 
				ArmMotor, 
				BallPusher, 
				ArmEncoder, 
				BallSensor, 
				setpoints
			);
		}
		catch(Exception ex)
		{
			Logs.error("Failed to initialize main components of the robot", ex);
		}
	}
	
	private static void initializeEncoders()
	{
		try
		{
			RightDriveEncoder = new Encoder(0, 1);
			LeftDriveEncoder = new Encoder(2, 3);
		}
		catch(Exception ex)
		{
			Logs.error("Failed to initialze encoders", ex);
		}
	}
}
