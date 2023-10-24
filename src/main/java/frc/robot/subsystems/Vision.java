package frc.robot.Subsystems;

import java.io.IOException;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;
import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;


import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Vision extends SubsystemBase
{
    PhotonPoseEstimator arCamPoseEstimator;
    PhotonCamera arCamera;
    AprilTagFieldLayout aprilTagFieldLayout;
        
    public Vision()
    {
        arCamera = new PhotonCamera("AR1");
        try 
        {
          aprilTagFieldLayout = AprilTagFieldLayout.loadFromResource(AprilTagFields.k2023ChargedUp.m_resourceFile);
        } 
        catch (IOException e) 
        {
          e.printStackTrace();
        }
        arCamPoseEstimator = new PhotonPoseEstimator(aprilTagFieldLayout, PoseStrategy.CLOSEST_TO_REFERENCE_POSE, arCamera, Constants.VisionConstants.robotToARCam);
    }

    public Pose2d getNearestAlignPose (Pose2d currentPose, String targetType)
    {
        //Determines wether the nearest left cone, mid cube, or right cone will be returned
        int searchRow = 0;
        switch(targetType)
        {
        case "leftCone":
            searchRow = 0;
        case "midCube":
            searchRow = 1;
        case "rightCone":
            searchRow = 2;
        default :
            searchRow = 1;
        }

        System.out.println("searc row" + searchRow);
        List<Pose2d> poseList = new ArrayList<>(Arrays.asList(Constants.VisionConstants.alignArray[searchRow]));
        return currentPose.nearest(poseList);
    }
    
    

    public Optional<EstimatedRobotPose> getPoseFromARCamCamera(Pose2d referencePose) 
    {
      arCamPoseEstimator.setReferencePose(referencePose);
      return arCamPoseEstimator.update();
    }
    
    
}
