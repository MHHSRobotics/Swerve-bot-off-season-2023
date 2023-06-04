package frc.robot.commands;

import java.util.Optional;

import org.photonvision.EstimatedRobotPose;

import frc.robot.subsystems.*;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.CommandBase;


public class AddVisionMeasurement extends CommandBase {

  Swerve swerve;
  Vision vision;
  Pose2d estimatedPose;
  double timestamp;

  public AddVisionMeasurement(Swerve swerve, Vision vision) {
    this.vision = vision;
    this.swerve = swerve;
    addRequirements(vision);
    }

  @Override
  public void initialize() {
  }
  
  @Override
  public void execute() {
    Optional<EstimatedRobotPose> arCamResult = vision.getPoseFromARCamCamera(swerve.getPose());


    if (arCamResult.isPresent()) {
      estimatedPose = arCamResult.get().estimatedPose.toPose2d();
      timestamp = arCamResult.get().timestampSeconds;
      swerve.addVisionMeasurement(estimatedPose, timestamp);
    }
    }
    
    

  @Override
  public boolean isFinished() {
    return false;
  }
}
