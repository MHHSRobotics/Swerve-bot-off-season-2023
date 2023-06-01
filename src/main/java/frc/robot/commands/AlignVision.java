package frc.robot.commands;

import frc.robot.Constants;
import frc.robot.subsystems.Swerve;
import frc.robot.subsystems.Vision;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;

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
        forwardPID = new PIDController(Constants.SwerveConstants.driveKP, Constants.SwerveConstants.driveKI, Constants.SwerveConstants.driveKD);
        horizontalPID = new PIDController(Constants.SwerveConstants.driveKP, Constants.SwerveConstants.driveKI, Constants.SwerveConstants.driveKD);
        rotationalPID = new PIDController(Constants.SwerveConstants.angleKP, Constants.SwerveConstants.angleKI, Constants.SwerveConstants.angleKD);
        addRequirements(swerve);   
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