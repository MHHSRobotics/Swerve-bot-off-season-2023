// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.hal.SimDevice;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;



import static frc.robot.Constants.Swerve.kModuleTranslations;;;


public class FieldSim extends SubsystemBase{
  private final Swerve m_swerveDrive;
  private double m_simYaw;
  SimDevice gyroSim = SimDevice.create("navX-Sensor[0]", SPI.Port.kMXP.value);



  private final Field2d m_field2d = new Field2d();


  private Pose2d[] m_swerveModulePoses = {
          new Pose2d(),
          new Pose2d(),
          new Pose2d(),
          new Pose2d()
  };

  public FieldSim(Swerve s_Swerve) {
    m_swerveDrive = s_Swerve;
   
  }

  public void initSim() {}

  public Field2d getField2d() {
    return m_field2d;
  }

  private void updateRobotPoses() {
    m_field2d.setRobotPose(m_swerveDrive.swerveOdometry.getPoseMeters());

    
    for (int i = 0; i < kModuleTranslations.length; i++) {
      Translation2d updatedPositions =
              kModuleTranslations[i]
                      .rotateBy(m_swerveDrive.swerveOdometry.getPoseMeters().getRotation())
                      .plus(m_swerveDrive.swerveOdometry.getPoseMeters().getTranslation());
      m_swerveModulePoses[i] =
              new Pose2d(
                      updatedPositions,
                      new Rotation2d(m_swerveDrive.swerveMods[i].angleEncoder.getPosition())
                              .plus(m_swerveDrive.getYaw()));
    }

    m_field2d.getObject("Swerve Modules").setPoses(m_swerveModulePoses);
  }

  @Override
  public void periodic() {
    updateRobotPoses();

    if (RobotBase.isSimulation()) simulationPeriodic();

    SmartDashboard.putData("Field2d", m_field2d);
  }
  
  
}
