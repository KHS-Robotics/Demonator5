package org.usfirst.frc.team4342.robot;

import org.usfirst.frc.team4342.robot.components.Repository;

import edu.wpi.first.wpilibj.CameraServer;
import edu.wpi.first.wpilibj.IterativeRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import org.usfirst.frc.team4342.api.autonomous.AutoRoutinesRunner;
import org.usfirst.frc.team4342.api.logging.SmartDashboardUpdater;
import org.usfirst.frc.team4342.api.multithreading.ComponentRunner;
import org.usfirst.frc.team4342.api.multithreading.ShootingComponent;
import org.usfirst.frc.team4342.api.multithreading.TankDriveComponent;

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
    /**
     * This function is run when the robot is first started up and should be
     * used for any initialization code.
     */
	@Override
    public void robotInit() 
    {
		Repository.initializeAll();
		SmartDashboardUpdater.startUpdating(Repository.Log, Repository.ConsoleLog);
		
		ComponentRunner.startAutomaticMode(new TankDriveComponent(Repository.TankDrive));
    	ComponentRunner.startAutomaticMode(new ShootingComponent(Repository.Shooter));
    	
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
		Repository.DriveTrain.setCoastMode();
		Repository.DriveTrain.enable();
    }
	
	@Override
	public void teleopPeriodic()
	{
		if(Repository.SwitchBox.getRawButton(9))
		{
			Repository.Navx.reset();
		}
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
		
		int def = (int)SmartDashboard.getNumber("RoutineDefense");
		
		if(def >= 2 && def <= 5)
		{
			AutoRoutinesRunner.setRoutineData(
				1,//(int)SmartDashboard.getNumber("RoutineStart"), 
				def,
				2,//(int)SmartDashboard.getNumber("RoutinePosition"),
				0,//(int)SmartDashboard.getNumber("RoutineGoal"), 
				0//(int)SmartDashboard.getNumber("RoutineFinish")
			);
		}
		else if(def == 0)
		{
			AutoRoutinesRunner.setRoutineData(
				1,//(int)SmartDashboard.getNumber("RoutineStart"), 
				3,
				2,//(int)SmartDashboard.getNumber("RoutinePosition"),
				0,//(int)SmartDashboard.getNumber("RoutineGoal"), 
				0//(int)SmartDashboard.getNumber("RoutineFinish")
			);
		}
		else
		{
			AutoRoutinesRunner.setRoutineData(
				0,//(int)SmartDashboard.getNumber("RoutineStart"), 
				0,
				0,//(int)SmartDashboard.getNumber("RoutinePosition"),
				0,//(int)SmartDashboard.getNumber("RoutineGoal"), 
				0//(int)SmartDashboard.getNumber("RoutineFinish")
			);
		}
		
	}
}

