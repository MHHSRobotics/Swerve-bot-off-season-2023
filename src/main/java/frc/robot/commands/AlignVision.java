package frc.robot.commands;

import frc.robot.Constants;
import frc.robot.subsystems.Swerve;
import frc.robot.subsystems.Vision;

import java.io.Console;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
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
public class AlignVision extends CommandBase {    
    
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

    public AlignVision(Swerve swerve, Vision vision, String scoreType)
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
     * 
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
     * 
     * Calculates speed in forward, horizontal, and rotational directions. 
     * Causes robot to eventually align in given position based upon the determined RobotPose. 
     */
    @Override
    public void execute() {
        /* Gets position of robot */
        robotPose = swerve.getPose();
        /*Calculates speeds to reach the align position */
        forwardSpeed = forwardPID.calculate(robotPose.getX(), targetPose.getX());
        horizontalSpeed = horizontalPID.calculate(robotPose.getY(), targetPose.getY());
        
        double currentDegrees = robotPose.getRotation().getDegrees()%360; 
        double targetDegrees = (targetPose.getRotation().getDegrees()+180)%360; 
        double degreeDiff = Math.abs(targetDegrees - currentDegrees);
        if(degreeDiff > 180)
        {
            double tempDegrees = targetDegrees;
            targetDegrees = currentDegrees;
            currentDegrees = tempDegrees;
        }
        rotationalSpeed = rotationalPID.calculate(
        (currentDegrees),
        (targetDegrees));
       
        if(Math.abs(((targetPose.getRotation().getDegrees()+180)%360)-(robotPose.getRotation().getDegrees()%360)) > 180 && rotationalSpeed > 0)
       {
        rotationalSpeed *= -1;
       }
        /* Inputs speed into drivetrain */
        swerve.drive(
            new Translation2d(-forwardSpeed, -horizontalSpeed), 
            rotationalSpeed, 
            true, 
            true
            
        );
        
    }

    @Override 
    public boolean isFinished()
    {
        if(Math.abs(horizontalSpeed) == 0 && Math.abs(forwardSpeed) == 0 && Math.abs(rotationalSpeed) == 0 )
        {
            return true;
        }
        return false;
    }
}