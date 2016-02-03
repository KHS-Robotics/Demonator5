 
package org.usfirst.frc.team4342.robot;

import org.usfirst.frc.team4342.robot.components.Repository;

import edu.wpi.first.wpilibj.IterativeRobot;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import org.usfirst.frc.team4342.api.logging.SmartDashboardUpdater;
import org.usfirst.frc.team4342.api.multithreading.ComponentRunner;
import org.usfirst.frc.team4342.api.multithreading.CompressorComponent;
import org.usfirst.frc.team4342.api.multithreading.ShootingComponent;
import org.usfirst.frc.team4342.api.multithreading.TankDriveComponent;
import org.usfirst.frc.team4342.api.pid.PID;

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
	private PID pid;
	
	private SendableChooser pChooser, iChooser, dChooser;
	private long numLoops;
	
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
		
		pid = new PID();
		
		pChooser = new SendableChooser();
		pChooser.addObject("P-Drive", 0.0);
		SmartDashboard.putData("Drive P Value", pChooser);
		iChooser = new SendableChooser();
		iChooser.addObject("I-Drive", 0.0);
		SmartDashboard.putData("Drive I Value", iChooser);
		dChooser = new SendableChooser();
		dChooser.addObject("D-Drive", 0.0);
		SmartDashboard.putData("Drive D Value", dChooser);
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
		
		if (Repository.SwitchBox.getRawButton(1))
		{
			pid.setSetpoint(0.0);
			pid.turnOn();
		}
		else
		{
			pid.turnOff();
		}
		
		if(numLoops % 10 == 0)
		{
			pid.setP(Integer.parseInt(SmartDashboard.getData("Drive P Value").toString()));
			pid.setI(Integer.parseInt(SmartDashboard.getData("Drive P Value").toString()));
			pid.setD(Integer.parseInt(SmartDashboard.getData("Drive P Value").toString()));
		}
		
		numLoops++;
    }
    
	/**
	 * This function is called when the robot is first starting disabled
	 */
    @Override
    public void disabledInit()
    {
    	numLoops = 0L;
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

