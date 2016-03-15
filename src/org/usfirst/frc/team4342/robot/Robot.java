package org.usfirst.frc.team4342.robot;

import org.usfirst.frc.team4342.robot.components.Repository;

import edu.wpi.first.wpilibj.IterativeRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import org.usfirst.frc.team4342.api.autonomous.AutoRoutine;
import org.usfirst.frc.team4342.api.autonomous.AutoRoutineLoader;
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
	private AutoRoutine selectedAutoRoutine;
	
    /**
     * This function is run when the robot is first started up and should be
     * used for any initialization code.
     */
	@Override
    public void robotInit() 
    {
		Repository.initializeAll();
		SmartDashboardUpdater.startUpdating(Repository.Log, Repository.ConsoleLog);

		selectedAutoRoutine = AutoRoutineLoader.getAutoRoutine("/home/lvuser/AutoRoutine.txt");
		
		ComponentRunner.startAutomaticMode(new TankDriveComponent(Repository.TankDrive));
    	ComponentRunner.startAutomaticMode(new ShootingComponent(Repository.Shooter));
		
		PIDTuner.startUpdating();
    }
	
	/**
	 * This function is run when the robot is first starting autonomous
	 */
	@Override
    public void autonomousInit() 
    {
    	Repository.DriveTrain.setBrakeMode();
    	Repository.DriveTrain.enable();
    	
    	selectedAutoRoutine = AutoRoutineLoader.getAutoRoutine((int)SmartDashboard.getNumber("Auto-Routine"));
    	
    	AutoRoutinesRunner.reset();
    }

    /**
     * This function is called periodically during autonomous
     */
	@Override
    public void autonomousPeriodic() 
    {
    	AutoRoutinesRunner.execute(selectedAutoRoutine);
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
	}
}

