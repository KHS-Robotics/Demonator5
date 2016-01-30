 
package org.usfirst.frc.team4342.robot;

import org.usfirst.frc.team4342.robot.components.Repository;

import edu.wpi.first.wpilibj.IterativeRobot;

import org.usfirst.frc.team4342.api.logging.SmartDashboardUpdater;
import org.usfirst.frc.team4342.api.multithreading.ComponentRunner;
import org.usfirst.frc.team4342.api.multithreading.CompressorComponent;
import org.usfirst.frc.team4342.api.multithreading.ShootingComponent;
import org.usfirst.frc.team4342.api.multithreading.TankDriveComponent;

/**
 * FRC Team 4342 (Kennett High School Demon Robotics) Robot Code for Stronghold.
 * 
 * @author Brice Chapman
 * @author Ernie Wilson
 * @author Katie Schuetz
 * @author Payton DuLong
 * @author Shakti Das
 */
public class Robot extends IterativeRobot 
{
	private CompressorComponent cc;
	private TankDriveComponent tdc;
	private ShootingComponent sc;
	
    /**
     * This function is run when the robot is first started up and should be
     * used for any initialization code.
     */
	@Override
    public void robotInit() 
    {
		Repository.initializeAll();
		
		SmartDashboardUpdater.startUpdating(Repository.Log, Repository.ConsoleLog);
		
		cc = new CompressorComponent(Repository.Compressor);
		ComponentRunner.startAutomaticMode(cc);
		
		tdc = new TankDriveComponent(Repository.TankDrive, 2);
		
		sc = new ShootingComponent(Repository.Shooter);
    }
	
	/**
	 * This function is run when the robot is first starting autonomous
	 */
	@Override
    public void autonomousInit() 
    {
    	Repository.DriveTrain.setBrakeMode();
    }

    /**
     * This function is called periodically during autonomous
     */
	@Override
    public void autonomousPeriodic() 
    {
    	
    }
	
	/**
	 * This function is run when the robot is first starting operator control
	 */
	@Override
    public void teleopInit()
    {
		Repository.DriveTrain.setBrakeMode();
		
    	ComponentRunner.startAutomaticMode(tdc);
    	ComponentRunner.startAutomaticMode(sc);
    }

    /**
     * This function is called periodically during operator control
     */
	@Override
    public void teleopPeriodic() 
    {
		if(Repository.SwitchBox.getRawButton(2) && cc.isRunning())
				ComponentRunner.stopAutomaticMode(cc);
		else if(!cc.isRunning())
				ComponentRunner.startAutomaticMode(cc);
		
		if(Repository.SwitchBox.getRawButton(9))
		{
			ComponentRunner.stopAutomaticMode(tdc);
			ComponentRunner.stopAutomaticMode(sc);
			ComponentRunner.stopAutomaticMode(cc);
		}
    }
    
	/**
	 * This function is called when the robot is first starting disabled
	 */
    @Override
    public void disabledInit()
    {
    	Repository.DriveTrain.setCoastMode();
    	ComponentRunner.stopAutomaticMode(tdc);
    	ComponentRunner.stopAutomaticMode(sc);
    }
    
    /**
     * This function is called periodically during disabled
     */
    @Override
    public void disabledPeriodic()
    {
    	
    }
}

