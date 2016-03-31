package org.usfirst.frc.team4342.robot;

import org.usfirst.frc.team4342.robot.multithreading.ComponentRunner;
import org.usfirst.frc.team4342.robot.multithreading.ShootingComponent;
import org.usfirst.frc.team4342.robot.multithreading.TankDriveComponent;

import edu.wpi.first.wpilibj.IterativeRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import org.usfirst.frc.team4342.api.autonomous.AutoRoutinesRunner;
import org.usfirst.frc.team4342.api.autonomous.AutoValuesDeserializer;
import org.usfirst.frc.team4342.api.autonomous.AutoValuesSerializer;
import org.usfirst.frc.team4342.api.autonomous.RoutineData;
import org.usfirst.frc.team4342.api.logging.SmartDashboardUpdater;
import org.usfirst.frc.team4342.api.shooter.arm.PIDTuner;

/**
 * FRC Team 4342 (Kennett High School Demon Robotics) Robot Code for Stronghold.
 * 
 * @see edu.wpi.first.wpilibj.IterativeRobot
 * @see edu.wpi.first.wpilibj.RobotBase
 * 
 * @author Brian Lucas
 * @author Ernie Wilson
 * @author Katie Schuetz
 * @author Payton DuLong
 * @author Shakti Das
 */
public class Robot extends IterativeRobot 
{	
	private TankDriveComponent tdc;
	private ShootingComponent sc;
	private boolean attemptedRestartShooter, attemptedRestartDrive;
	
    /**
     * This function is run when the robot is first started up and should be
     * used for any initialization code.
     */
	@Override
    public void robotInit() 
    {
		Repository.initializeAll();
		SmartDashboardUpdater.startUpdating(Repository.Log, Repository.ConsoleLog);
		
		tdc = new TankDriveComponent(Repository.TankDrive);
		sc = new ShootingComponent(Repository.Shooter);
		
		ComponentRunner.startAutomaticMode(tdc);
    	ComponentRunner.startAutomaticMode(sc);
    	
    	// Used for "on the spot" tuning of the arm PID
    	PIDTuner.startUpdating(4);
    }
	
	/**
	 * This function is run when the robot is first starting autonomous
	 */
	@Override
    public void autonomousInit() 
    {
    	Repository.DriveTrain.setBrakeMode();
    	Repository.DriveTrain.setVoltageRampRate(1023);
    	Repository.DriveTrain.enable();
    	
    	Repository.ArmController.enablePID();
    	Repository.ShooterController.enablePID();
    	
    	int start = (int)SmartDashboard.getNumber("RoutineStart");
		int defense = (int)SmartDashboard.getNumber("RoutineDefense");
		int position = (int)SmartDashboard.getNumber("RoutinePosition");
		int goal = (int)SmartDashboard.getNumber("RoutineGoal");
		int finish = (int)SmartDashboard.getNumber("RoutineFinish");
		
		if(start == 0 && defense == 0 && position == 0 && goal == 0 && finish == 0)
		{
			Repository.Logs.warning("No auto values set, loading defaults...");
			RoutineData d = AutoValuesDeserializer.load();
			
			AutoRoutinesRunner.setRoutineData(
				d.getStart(),
				d.getDefense(),
				d.getPosition(),
				d.getGoal(),
				d.getFinish()
			);
			
			if(d != null)
				Repository.Logs.info("Successfully loaded default auto; see log for more info");
		}
    	
		if(AutoRoutinesRunner.getRoutineData() != null)
		{
			AutoRoutinesRunner.reset();
	    	Repository.Timer.start();
		}
    }

    /**
     * This function is called periodically during autonomous
     */
	@Override
    public void autonomousPeriodic() 
    {
		if(AutoRoutinesRunner.getRoutineData() != null)
			AutoRoutinesRunner.execute();
    }
	
	/**
	 * This function is run when the robot is first starting operator control
	 */
	@Override
    public void teleopInit()
    {
		Repository.DriveTrain.setVoltageRampRate(64);
		Repository.DriveTrain.setCoastMode();
		Repository.DriveTrain.enable();
    }
	
	@Override
	public void teleopPeriodic()
	{
		// Only used when testing
		if(Repository.SwitchBox.getRawButton(9))
		{
			Repository.Navx.reset();
		}
		
		checkShooter();
		checkDrive();
	}
	
	/**
	 * This function is run when the robot is first starting disabled
	 */
	@Override
	public void disabledInit()
	{
		SmartDashboard.putNumber("Yaw-Offset", 0.0);
	}
	
	/**
	 * This function is called periodically during disabled
	 */
	@Override
	public void disabledPeriodic()
	{
		Repository.TankDrive.setYawOffset(SmartDashboard.getNumber("Yaw-Offset"));
		
		int start = (int)SmartDashboard.getNumber("RoutineStart");
		int defense = (int)SmartDashboard.getNumber("RoutineDefense");
		int position = (int)SmartDashboard.getNumber("RoutinePosition");
		int goal = (int)SmartDashboard.getNumber("RoutineGoal");
		int finish = (int)SmartDashboard.getNumber("RoutineFinish");
		
		AutoRoutinesRunner.setRoutineData(
			start, 
			defense,
			position,
			goal, 
			finish
		);
		
		if(Repository.SwitchBox.getRawButton(9))
		{
			AutoValuesSerializer.save(
				start, 
				defense,
				position,
				goal, 
				finish
			);
		}
	}
	
	// This probably wont do much considering if the thread does crash,
	// then the "zero" spot is erroneous. But if the arm is all the way
	// down then maybe we can low bar and shoot low
	private void checkShooter()
	{
		if(!sc.isRunning() && !attemptedRestartShooter)
		{
			Repository.Logs.warning("Shooter has crashed, attempting restart");
			ComponentRunner.startAutomaticMode(sc);
			attemptedRestartShooter = true;
		}
		else if(sc.isRunning() && attemptedRestartShooter)
		{
			Repository.Logs.warning("Shooter has been successfully restarted");
		}
		else if(!sc.isRunning())
		{
			Repository.Logs.warning("Shooter has crashed again, NOT restarting; see logs for details");
		}
	}
	
	private void checkDrive()
	{
		if(!tdc.isRunning() && !attemptedRestartDrive)
		{
			Repository.Logs.warning("Drive has crashed, attempting restart");
			ComponentRunner.startAutomaticMode(sc);
			attemptedRestartDrive = true;
		}
		else if(tdc.isRunning() && attemptedRestartDrive)
		{
			Repository.Logs.warning("Drive has been successfully restarted");
		}
		else if(!tdc.isRunning())
		{
			Repository.Logs.warning("Drive has crashed again, NOT restarting; see logs for details");
		}
	}
}

