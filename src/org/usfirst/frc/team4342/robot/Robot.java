package org.usfirst.frc.team4342.robot;

import org.usfirst.frc.team4342.robot.components.Repository;

import edu.wpi.first.wpilibj.IterativeRobot;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import org.usfirst.frc.team4342.api.autonomous.AutoRoutine;
import org.usfirst.frc.team4342.api.autonomous.AutoRoutineLoader;
import org.usfirst.frc.team4342.api.autonomous.AutoRoutinesRunner;
import org.usfirst.frc.team4342.api.logging.NavXLogger;
import org.usfirst.frc.team4342.api.logging.SmartDashboardUpdater;
import org.usfirst.frc.team4342.api.multithreading.ComponentRunner;
import org.usfirst.frc.team4342.api.multithreading.ShootingComponent;
import org.usfirst.frc.team4342.api.multithreading.TankDriveComponent;

/**
 * FRC Team 4342 (Kennett High School Demon Robotics) Robot Code for Stronghold.
 * 
 * @see edu.wpi.first.wpilibj.IterativeRobot;
 * 
 * @author Ernie Wilson
 * @author Katie Schuetz
 * @author Payton DuLong
 * @author Shakti Das
 */
public class Robot extends IterativeRobot 
{
	private TankDriveComponent tdc;
	private ShootingComponent sc;
	
	private AutoRoutine selectedAutoRoutine;
	
    /**
     * This function is run when the robot is first started up and should be
     * used for any initialization code.
     */
	@Override
    public void robotInit() 
    {
		SmartDashboard.putNumber("Goal-Dist", 0.0);
		SmartDashboard.putNumber("Goal-Ang", 0.0);
		
		SmartDashboard.putNumber("Arm-P-Up", 0.0);
		SmartDashboard.putNumber("Arm-I-Up", 0.0);
		SmartDashboard.putNumber("Arm-D-Up", 0.0);
		SmartDashboard.putNumber("Arm-P-Down", 0.0);
		SmartDashboard.putNumber("Arm-I-Down", 0.0);
		SmartDashboard.putNumber("Arm-D-Down", 0.0);
		
		Repository.initializeAll();
		SmartDashboardUpdater.startUpdating(Repository.Log, Repository.ConsoleLog);
		
		tdc = new TankDriveComponent(Repository.TankDrive);
		sc = new ShootingComponent(Repository.Shooter);
		
		new Solenoid(2).set(true);
		new Solenoid(3).set(true);
		
		PIDTuner.startUpdating();
    }
	
	/**
	 * This function is run when the robot is first starting autonomous
	 */
	@Override
    public void autonomousInit() 
    {
    	Repository.DriveTrain.setBrakeMode();
    	
    	ComponentRunner.startAutomaticMode(tdc);
    	ComponentRunner.startAutomaticMode(sc);
    	
    	selectedAutoRoutine = AutoRoutineLoader.getAutoRoutine("/home/lvuser/AutoRoutine.txt");
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
		
    	ComponentRunner.startAutomaticMode(tdc);
    	ComponentRunner.startAutomaticMode(sc);
    }

    /**
     * This function is called periodically during operator control
     */
	@Override
    public void teleopPeriodic() 
    {
		if(Repository.SwitchBox.getRawButton(8))
			NavXLogger.startLogging(Repository.Navx, Repository.DriveTrain);
		else
			NavXLogger.stopLogging();
    }
    
	/**
	 * This function is called when the robot is first starting disabled
	 */
    @Override
    public void disabledInit()
    {
    	//Repository.DriveTrain.setCoastMode();
    	ComponentRunner.stopAutomaticMode(tdc);
    	ComponentRunner.stopAutomaticMode(sc);
    }

    
    /**
     * This function is called periodically during disabled
     */
    @Override
    public void disabledPeriodic()
    {
    	// Nothing we need to do in here
    }
}

