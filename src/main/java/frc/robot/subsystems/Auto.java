package frc.robot.Subsystems;

import java.util.ArrayList;
import java.util.HashMap;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.auto.PIDConstants;
import com.pathplanner.lib.auto.SwerveAutoBuilder;


import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;

/*This is an example auto subsystem, created by 5137, that can be used with PathPlanner (an frc community project for auto)*/
public class Auto {

    private Swerve swerveDrive;
    private SwerveAutoBuilder swerveAutoBuilder;

    /*The path group used to run an auto sequence from PathPlanner */
    private ArrayList<PathPlannerTrajectory> exampleAuto = (ArrayList<PathPlannerTrajectory>) PathPlanner.loadPathGroup("examplePath", new PathConstraints(4, 3));


    /**
     * Auto system that will run auto paths using pathplanner
     * @param swerveDrive the swerve subsystem that the auto system should control 
     */
    public Auto(Swerve swerveDrive)
    {
        /*The swerve subsystem which is initialized for use by the auto system */
        this.swerveDrive = swerveDrive; 
        
        /*The autobuilder which uses various inputs to control the robot during auto */
        swerveAutoBuilder = new SwerveAutoBuilder(
        swerveDrive::getPose, //Uses the getPose method of the swerve class to determine where the robot is
        swerveDrive::resetOdometry, //Uses the resetOdometery method to reset the relative position of the robot at the start of auto
        Constants.SwerveConstants.swerveKinematics, //The dimensions of our swerve drivetrain 
        new PIDConstants(Constants.SwerveConstants.driveKP, Constants.SwerveConstants.driveKI, Constants.SwerveConstants.driveKD), //PID Values for turning and driving in Auto
        new PIDConstants(Constants.SwerveConstants.angleKP, Constants.SwerveConstants.angleKI, Constants.SwerveConstants.angleKD),
        swerveDrive::setModuleStates, //What the auto builder uses to physically control the robot 
        new HashMap<>(), //A hashmap that contains both a String value and a Command value so a specific command will execute when a string has been placed in a Pathplanner marker
        true, //Mirrors the path based on alliance color 
         swerveDrive); //The required system to run auto -- makes sure that it uses the command scheduler properly to prevent any driving interferences
    }

    /**
     * A method used to return an auto path to be scheduled in Auto mode 
     * This method can be edited to allow for selection of different path groups
     * @return a runnable auto command for scheduling
    */
    public Command getAuto()
    {
        /*Call the full auto method to recieve a command that will run an autopath */
        return swerveAutoBuilder.fullAuto(exampleAuto); 
    }
  
}
