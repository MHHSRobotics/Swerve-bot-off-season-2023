package frc.robot.subsystems;

import java.io.IOException;
import java.util.List;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Vision extends SubsystemBase
{
    private AprilTagFieldLayout aprilTagFieldLayout; 
    private List<Pose2d> aprilTagPoses;
    private double scoringDistance = Units.inchesToMeters(36); 
    public Vision()
    {
        /* Try and Catch to Load the Field from a file */
        try 
        {
            aprilTagFieldLayout = AprilTagFieldLayout.loadFromResource(AprilTagFields.k2023ChargedUp.m_resourceFile);
        } 
        catch (IOException e) 
        {
            e.printStackTrace();
        }

        /* Fills the List with Pose2d of the AprilTags */
        for(int i = 1; i <= aprilTagFieldLayout.getTags().size(); i++)
        {
            aprilTagPoses.add(aprilTagFieldLayout.getTagPose(i).get().toPose2d().transformBy(new Transform2d(new Translation2d(0, scoringDistance), new Rotation2d(Math.PI))));
            
        }
    }

    public Pose2d getNearestAprilTagPose(Pose2d currentPose)
    {
        return currentPose.nearest(aprilTagPoses);
    }
}