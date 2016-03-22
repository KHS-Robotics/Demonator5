package org.usfirst.frc.team4342.robot;

import org.usfirst.frc.team4342.robot.components.Repository;
import org.usfirst.frc.team4342.robot.multithreading.ComponentRunner;
import org.usfirst.frc.team4342.robot.multithreading.ShootingComponent;
import org.usfirst.frc.team4342.robot.multithreading.TankDriveComponent;

import edu.wpi.first.wpilibj.IterativeRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import org.usfirst.frc.team4342.api.autonomous.AutoRoutinesRunner;
import org.usfirst.frc.team4342.api.logging.SmartDashboardUpdater;

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
    	
    	//PIDTuner.startUpdating();
    }
	
	/**
	 * This function is run when the robot is first starting autonomous
	 */
	@Override
    public void autonomousInit() 
    {
    	Repository.DriveTrain.setBrakeMode();
    	Repository.DriveTrain.enable();
    	
    	Repository.ArmController.enablePID();
    	Repository.ShooterController.enablePID();
    	    	
    	AutoRoutinesRunner.reset();
    	Repository.Timer.start();
    }

    /**
     * This function is called periodically during autonomous
     */
	@Override
    public void autonomousPeriodic() 
    {
    	AutoRoutinesRunner.execute();
    }
	
	/**
	 * This function is run when the robot is first starting operator control
	 */
	@Override
    public void teleopInit()
    {
		Repository.DriveTrain.setVoltageRampRate(24);
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
		
		AutoRoutinesRunner.setRoutineData(
			(int)SmartDashboard.getNumber("RoutineStart"), 
			(int)SmartDashboard.getNumber("RoutineDefense"),
			(int)SmartDashboard.getNumber("RoutinePosition"),
			(int)SmartDashboard.getNumber("RoutineGoal"), 
			(int)SmartDashboard.getNumber("RoutineFinish")
		);
	}
	
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
			Repository.Logs.warning("Shooter has crashed again; see logs for details");
		}
	}
	
	private void checkDrive()
	{
		if(!tdc.isRunning() && !attemptedRestartDrive)
		{
			Repository.Logs.warning("Shooter has crashed, attempting restart");
			ComponentRunner.startAutomaticMode(sc);
			attemptedRestartDrive = true;
		}
		else if(tdc.isRunning() && attemptedRestartDrive)
		{
			Repository.Logs.warning("Shooter has been successfully restarted");
		}
		else if(!tdc.isRunning())
		{
			Repository.Logs.warning("Shooter has crashed again, NOT restarting; see logs for details");
		}
	}
}

