package frc.robot.commands;

import frc.robot.Constants;
import frc.robot.subsystems.Swerve;
import frc.robot.subsystems.Vision;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class AlignVision extends CommandBase {    
    private Swerve swerve; 
    private Vision vision;  
    private Pose2d targetPose; 
    private Pose2d robotPose;
    private double forwardSpeed;
    private double horizontalSpeed;
    private double rotationalSpeed; 
    //private PIDController forwardPID;
    private ProfiledPIDController forwardPID;
    private ProfiledPIDController horizontalPID;
    private ProfiledPIDController rotationalPID;
   
    private String scoreType;

    public AlignVision(Swerve swerve, Vision vision, String scoreType)
    {    
        this.swerve = swerve;
        this.vision = vision;
        this.scoreType = scoreType;
        
        forwardPID = new ProfiledPIDController(
        Constants.SwerveConstants.driveKP, Constants.SwerveConstants.driveKI, Constants.SwerveConstants.driveKD,
        new Constraints(Constants.AutoConstants.kMaxSpeedMetersPerSecond, Constants.AutoConstants.kMaxAccelerationMetersPerSecondSquared));
        forwardPID.setTolerance(Constants.VisionConstants.alignToleranceMeters);
        
        horizontalPID = new ProfiledPIDController(
        Constants.SwerveConstants.driveKP, Constants.SwerveConstants.driveKI, Constants.SwerveConstants.driveKD,
        new Constraints(Constants.AutoConstants.kMaxSpeedMetersPerSecond, Constants.AutoConstants.kMaxAccelerationMetersPerSecondSquared));
        horizontalPID.setTolerance(Constants.VisionConstants.alignToleranceMeters);

       
        rotationalPID = new ProfiledPIDController(
        Constants.SwerveConstants.angleKP, Constants.SwerveConstants.angleKI, Constants.SwerveConstants.angleKD,
        new Constraints(Constants.AutoConstants.kMaxAngularSpeedRadiansPerSecond, Constants.AutoConstants.kMaxAngularSpeedRadiansPerSecondSquared));
        rotationalPID.setTolerance(Constants.VisionConstants.alignToleranceRadians);

    }

    @Override
    public void initialize()
    {
        robotPose = swerve.getPose();
        targetPose = vision.getNearestAlignPose(robotPose, scoreType);
    }
    @Override
    public void execute() {
        robotPose = swerve.getPose();
        forwardSpeed = forwardPID.calculate(robotPose.getX(), targetPose.getX());
        horizontalSpeed = horizontalPID.calculate(robotPose.getY(), targetPose.getY());
        horizontalSpeed = rotationalPID.calculate(robotPose.getRotation().getRadians(), targetPose.getRotation().getRadians());
        
        /* Drive */
        swerve.drive(
            new Translation2d(forwardSpeed, horizontalSpeed), 
            rotationalSpeed, 
            true, 
            false
        );
    }

    @Override 
    public boolean isFinished()
    {
        if(horizontalSpeed == 0 || forwardSpeed == 0 || rotationalSpeed == 0)
        {
            return true;
        }
        return false;
    }
}