package org.usfirst.frc.team4342.robot;

import org.usfirst.frc.team4342.api.logging.PdpLogger;
import org.usfirst.frc.team4342.api.logging.RobotConsoleLogger;
import org.usfirst.frc.team4342.api.logging.RobotLogFactory;
import org.usfirst.frc.team4342.api.logging.SmartDashboardUpdater;
import org.usfirst.frc.team4342.api.drive.DriveTrain;
import org.usfirst.frc.team4342.api.drive.TankDrive;
import org.usfirst.frc.team4342.api.shooter.Shooter;
import org.usfirst.frc.team4342.api.shooter.ShooterController;
import org.usfirst.frc.team4342.api.shooter.arm.ArmController;

import edu.wpi.first.wpilibj.CANTalon;
import edu.wpi.first.wpilibj.CameraServer;
import edu.wpi.first.wpilibj.Counter;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.PIDSourceType;
import edu.wpi.first.wpilibj.PowerDistributionPanel;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

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
	
	public static edu.wpi.first.wpilibj.Timer Timer = new edu.wpi.first.wpilibj.Timer();
	
	public static Joystick DriveStick, ShooterStick, SwitchBox;
	
	public static CANTalon FrontRight, FrontLeft, MiddleRight,
							MiddleLeft, RearRight, RearLeft;
	
	public static DriveTrain DriveTrain;
	public static DoubleSolenoid Shifter;
	public static Encoder LeftDriveEncoder, RightDriveEncoder;
	public static AHRS Navx;
	
	public static CANTalon RightShooter, LeftShooter, ArmMotor, Accumulator;
	public static Solenoid BallPusher;
	public static Encoder ArmEncoder;
	public static Counter RightMotorCounter, LeftMotorCounter;
	public static DigitalInput BallSensor, TopLimitSwitch, BottomLimitSwitch;
	
	public static ArmController ArmController;
	public static ShooterController ShooterController;
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
			initializeSmartDashboard();
			
			try
			{
				CameraServer camera = CameraServer.getInstance();
		    	camera.setQuality(100);
		    	camera.startAutomaticCapture("cam0");
			}
			catch(Exception ex)
			{
				Logs.error("Failed to start USB Camera", ex);
			}
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
			ConsoleLog.warning("Failed to initialize log");
			Logs = new MultiLogger(new ILogger[] { ConsoleLog });
		}
		
		try
		{
			PdpLogger.start(new PowerDistributionPanel(), Log, ConsoleLog);
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
			FrontRight = new CANTalon(2);
			FrontLeft = new CANTalon(13);
			MiddleRight = new CANTalon(1);
			MiddleLeft = new CANTalon(14);
			RearRight = new CANTalon(0);
			RearLeft = new CANTalon(15);
			
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
			LeftShooter = new CANTalon(12);
			ArmMotor = new CANTalon(4);
			Accumulator = new CANTalon(11);
			
			RightShooter.enableBrakeMode(false);
			LeftShooter.enableBrakeMode(false);
			ArmMotor.enableBrakeMode(true);
			Accumulator.enableBrakeMode(false);
			
			RightShooter.enable();
			LeftShooter.enable();
			ArmMotor.enable();
			Accumulator.enable();
			
			TopLimitSwitch = new DigitalInput(8);
			BottomLimitSwitch = new DigitalInput(9);
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
			new Solenoid(2).set(true);
			new Solenoid(3).set(true);
			Shifter = new DoubleSolenoid(4, 5);
			BallPusher = new Solenoid(6);
		}
		catch(Exception ex)
		{
			Logs.error("Failed to initialize pneumatics", ex);
		}
	}
	
	private static void initializeEncoders()
	{
		try
		{
			RightDriveEncoder = new Encoder(1, 0);
			LeftDriveEncoder = new Encoder(2, 3);
			ArmEncoder = new Encoder(4, 5);
			RightMotorCounter = new Counter(6);
			LeftMotorCounter = new Counter(7);
			
			// 7.5 * PI / 128
			RightDriveEncoder.setDistancePerPulse(0.184);
			LeftDriveEncoder.setDistancePerPulse(0.184);
			RightDriveEncoder.setPIDSourceType(PIDSourceType.kDisplacement);
			LeftDriveEncoder.setPIDSourceType(PIDSourceType.kDisplacement);
			
			// 1 / (7*142)
			ArmEncoder.setDistancePerPulse(1);
			ArmEncoder.setPIDSourceType(PIDSourceType.kDisplacement);
			
			RightMotorCounter.setDistancePerPulse(0.05);
			LeftMotorCounter.setDistancePerPulse(0.05);
		}
		catch(Exception ex)
		{
			Logs.error("Failed to initialze encoders", ex);
		}
	}
	
	private static void initializeComponents()
	{
		try
		{
			TankDrive = new TankDrive(
				DriveStick, 
				DriveTrain, 
				Navx, 
				Shifter, 
				LeftDriveEncoder, 
				RightDriveEncoder
			);
			
			ArmController = new ArmController(ShooterStick, SwitchBox, ArmMotor, Accumulator, ArmEncoder, TopLimitSwitch, BottomLimitSwitch);
			ShooterController = new ShooterController(DriveStick, SwitchBox, RightShooter, LeftShooter, BallPusher, RightMotorCounter, LeftMotorCounter, BallSensor, ArmController);
			
			Shooter = new Shooter(ShooterController, ArmController);
		}
		catch(Exception ex)
		{
			Logs.error("Failed to initialize main components of the robot", ex);
		}
	}
	
	private static void initializeSmartDashboard()
	{
		SmartDashboard.putNumber("Arm-Setpoint", 0);
		
		SmartDashboard.putNumber("RoutineStart", 0);
		SmartDashboard.putNumber("RoutineDefense", 0);
		SmartDashboard.putNumber("RoutinePosition", 0);
		SmartDashboard.putNumber("RoutineGoal", 0);
		SmartDashboard.putNumber("RoutineFinish", 0);
		
		SmartDashboard.putNumber("Goal-Dist", 0.0);
		SmartDashboard.putNumber("Goal-Ang", 0.0);
		
		SmartDashboard.putNumber("Arm-P-Up", 0.0);
		SmartDashboard.putNumber("Arm-I-Up", 0.0);
		SmartDashboard.putNumber("Arm-D-Up", 0.0);
		SmartDashboard.putNumber("Arm-P-Down", 0.0);
		SmartDashboard.putNumber("Arm-I-Down", 0.0);
		SmartDashboard.putNumber("Arm-D-Down", 0.0);
		
		SmartDashboardUpdater.addJoystick("Drive", DriveStick);
		SmartDashboardUpdater.addJoystick("Shooter", ShooterStick);
		
		SmartDashboardUpdater.addTalon("FR", FrontRight);
		SmartDashboardUpdater.addTalon("FL", FrontLeft);
		SmartDashboardUpdater.addTalon("MR", MiddleRight);
		SmartDashboardUpdater.addTalon("ML", MiddleLeft);
		SmartDashboardUpdater.addTalon("RR", RearRight);
		SmartDashboardUpdater.addTalon("RL", RearLeft);
		SmartDashboardUpdater.addTalon("Accum", Accumulator);
		SmartDashboardUpdater.addTalon("Arm", ArmMotor);
		SmartDashboardUpdater.addTalon("Shooter-R", RightShooter);
		SmartDashboardUpdater.addTalon("Shooter-L", LeftShooter);
		
		SmartDashboardUpdater.addEncoder("Drive-R", RightDriveEncoder);
		SmartDashboardUpdater.addEncoder("Drive-L", LeftDriveEncoder);
		SmartDashboardUpdater.addEncoder("Arm", ArmEncoder);
		
		SmartDashboardUpdater.addDigitalInput("Arm-Top", TopLimitSwitch);
		SmartDashboardUpdater.addDigitalInput("Arm-Bot", BottomLimitSwitch);
		
		SmartDashboardUpdater.addCounter("Shooter-R", RightMotorCounter);
		SmartDashboardUpdater.addCounter("Shooter-L", LeftMotorCounter);
		
		SmartDashboardUpdater.addDefenseState("RampParts-State", TankDrive.getRampPartsState());
		SmartDashboardUpdater.addDefenseState("RoughTerrain-State", TankDrive.getRoughTerrainState());
		SmartDashboardUpdater.addDefenseState("Moat-State", TankDrive.getMoatState());
		SmartDashboardUpdater.addDefenseState("LowBar-State", TankDrive.getLowBarState());
		SmartDashboardUpdater.addDefenseState("RockWall-State", TankDrive.getRockWallState());
	}
}
