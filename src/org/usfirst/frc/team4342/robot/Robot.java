 
package org.usfirst.frc.team4342.robot;

import org.usfirst.frc.team4342.robot.components.Repository;

import edu.wpi.first.wpilibj.IterativeRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import static org.usfirst.frc.team4342.robot.components.Repository.Navx;

import org.usfirst.frc.team4342.api.drive.TankDrive;
import org.usfirst.frc.team4342.api.pnuematics.Compressor;

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
	private Compressor compressor;
	private TankDrive drive;
	
    /**
     * This function is run when the robot is first started up and should be
     * used for any initialization code.
     */
	@Override
    public void robotInit() 
    {
		Repository.initializeAll();
		
		compressor = new Compressor(
			Repository.Compressor,
			Repository.PressureSwitch
		);
		compressor.setAutomaticMode();
		
		drive = new TankDrive(
			Repository.DriveStick,
			Repository.DriveTrain,
			Navx,
			Repository.Shifter,
			Repository.Cylinder
		);
		TankDrive.setAutomaticMode(drive, 1, 2);
    }
	
	/**
	 * This function is run when the robot is first starting autonomous
	 */
	@Override
    public void autonomousInit() 
    {
    	
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
    	drive.setBrakeMode();
    }

    /**
     * This function is called periodically during operator control
     */
	@Override
    public void teleopPeriodic() 
    {
		
    }
    
	/**
	 * This function is called when the robot is first starting disabled
	 */
    @Override
    public void disabledInit()
    {
    	drive.setCoastMode();
    }
    
    /**
     * This function is called periodically during disabled
     */
    @Override
    public void disabledPeriodic()
    {
    	navxSmartDashboradTest();
    }
    
    public void navxSmartDashboradTest()
    {
    	SmartDashboard.putNumber("NavX-Yaw", Navx.getYaw());
    	SmartDashboard.putNumber("NavX-Pitch", Navx.getPitch());
    	SmartDashboard.putNumber("NavX-Roll", Navx.getRoll());
    	SmartDashboard.putNumber("NavX-Accel-X", Navx.getRawAccelX());
    	SmartDashboard.putNumber("NavX-Accel-Y", Navx.getRawAccelY());
    	SmartDashboard.putNumber("NavX-Accel-Z", Navx.getRawAccelZ());
    	SmartDashboard.putNumber("NavX-Mag-X", Navx.getRawMagX());
    	SmartDashboard.putNumber("NavX-Mag-Y", Navx.getRawMagY());
    	SmartDashboard.putNumber("NavX-Mag-Z", Navx.getRawMagZ());
    	SmartDashboard.putBoolean("NavX-Connected", Navx.isConnected());
    	SmartDashboard.putBoolean("NavX-Calibrating", Navx.isCalibrating());
    }
   
}

