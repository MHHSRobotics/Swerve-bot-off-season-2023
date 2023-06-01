package frc.robot.subsystems;

import java.io.IOException;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;


import edu.wpi.first.math.geometry.Pose2d;


import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Vision extends SubsystemBase
{
        
    public Vision()
    {
        
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
        }

        List<Pose2d> poseList = new ArrayList<>(Arrays.asList(Constants.VisionConstants.alignArray[searchRow]));
        return currentPose.nearest(poseList);
    }
    
    
/* 
    public Optional<EstimatedRobotPose> getPoseFromARCamCamera(Pose2d referencePose) 
    {
      arCamPoseEstimator.setReferencePose(referencePose);
      return arCamPoseEstimator.update();
    }
    */
    
}
