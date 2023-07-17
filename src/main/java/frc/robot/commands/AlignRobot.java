package frc.robot.commands;

import frc.robot.Constants;
import frc.robot.subsystems.Swerve;
import frc.robot.subsystems.Vision;

import java.io.Console;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.wpilibj2.command.CommandBase;

/**
 * Command class that when triggered will align the robot with the 
 * nearest scoring position based on controller selection.
 * 
 * Currently Aligns with XBOX 'A' being the nearest cube scoring spot,
 *  XBOX 'B' being the nearest right cone spot, 
 * and XBOX 'X' being the nearest left cone spot. 
 */
public class AlignRobot extends CommandBase {    
    
    private Swerve swerve; 
    private Vision vision;  
    
    private Pose2d targetPose; 
    private Pose2d robotPose;
    
    private double forwardSpeed;
    private double horizontalSpeed;
    private double rotationalSpeed; 

    private PIDController forwardPID;
    private PIDController horizontalPID;
    private PIDController rotationalPID;
   
    private String scoreType;

    public AlignRobot(Swerve swerve, Vision vision, String scoreType)
    {    
        this.swerve = swerve;
        this.vision = vision;
        this.scoreType = scoreType;
        
        /* Sets PID Controller gains and position acceptable tolerance/error */
        forwardPID = new PIDController(0.5,0,0);
        forwardPID.setTolerance(0.1);
        
        horizontalPID = new PIDController(0.5,0,0);
        horizontalPID.setTolerance(0.1);

        rotationalPID = new PIDController(.01, 0, 0);
        rotationalPID.setTolerance(3);

    }

    /**
     * The method that is called at the start of the command. 
     * Decides on target position based upon current position, when triggered,  and controller input. 
     */
    @Override
    public void initialize()
    {
        robotPose = swerve.getPose();
        targetPose = vision.getNearestAlignPose(robotPose, scoreType);
    }

    /**
     * Method called continously until command is finished. 
     * Calculates speed in forward, horizontal, and rotational directions. 
     * Causes robot to eventually align in given position based upon the determined RobotPose. 
     */
    @Override
    public void execute() {
        /* Updates position of robot */
        robotPose = swerve.getPose();
        /*Calculates speeds to reach the align position */
        forwardSpeed = forwardPID.calculate(robotPose.getX(), targetPose.getX());
        horizontalSpeed = horizontalPID.calculate(robotPose.getY(), targetPose.getY());
        rotationalSpeed = rotationalPID.calculate(getCurrentDegrees(),getTargetDegrees());
        /* Inputs speed into drivetrain */
        swerve.drive(
            new Translation2d(-forwardSpeed, -horizontalSpeed), 
            rotationalSpeed, 
            true, 
            true
        );
        
    }

    /**
     * Method called to check if command is finished.
     * Returns true if PID speeds are all 0. 
     */
    @Override 
    public boolean isFinished()
    {
        if(horizontalSpeed == 0 && forwardSpeed == 0 && rotationalSpeed == 0)
        {
            return true;
        }
        return false;
    }

    /**
     * Optimizes angle so robot does not over-rotate. 
     * @return robot position in degrees from 0-359.
     */
    private double getCurrentDegrees()
    {
        return robotPose.getRotation().getDegrees()%360;
    }

    /**
     * Optimizes angle so that the robot uses the shortest spin to align.
     * @return angle of target position in degrees from -179 to 180.
     */
    private double getTargetDegrees()
    {
        double targetDegrees = (targetPose.getRotation().getDegrees()+180)%360;
        if(Math.abs(targetDegrees-getCurrentDegrees())>180) 
        {
            return targetDegrees-=360; //If robot is spinning the "long way around" (>180*), correct target degrees to minimize excess.
        }
        return targetDegrees;
    }
}