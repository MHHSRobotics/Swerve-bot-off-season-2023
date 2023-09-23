package frc.robot;

import java.io.IOException;
import java.util.List;

import com.revrobotics.CANSparkMax.IdleMode;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import frc.lib.util.COTSFalconSwerveConstants;
import frc.lib.util.SwerveModuleConstants;


public final class Constants {
    public static final double stickDeadband = 0.1;
    public static final double alignToleranceMeters = 0.1;
    public static final double alignToleranceDegrees = 1;

    public static final class ElevatorConstants {
        public static final int motor1Port = 9;
        public static final int motor2Port = 10;

        public static final double kP = 1.0;
        public static final double kI = 0.0;
        public static final double kD = 0.0;

        public static final double maxVelo = 1.0;
        public static final double maxAccel = 1.0;
    }
    
    public static final class SwerveConstants {
        public static final boolean invertGyro = true; // Always ensure Gyro is CCW+ CW-

        public static final COTSFalconSwerveConstants chosenModule =  
            COTSFalconSwerveConstants.SDSMK4i(COTSFalconSwerveConstants.driveGearRatios.SDSMK4i_L2);

        /* Drivetrain Constants */
        public static final double trackWidth = Units.inchesToMeters(24.75);
        public static final double wheelBase = Units.inchesToMeters(24.75); 
        public static final double wheelCircumference = chosenModule.wheelCircumference;

        /* Swerve Kinematics 
         * No need to ever change this unless you are not doing a traditional rectangular/square 4 module swerve */
         public static final SwerveDriveKinematics swerveKinematics = new SwerveDriveKinematics(
            new Translation2d(wheelBase / 2.0, trackWidth / 2.0),
            new Translation2d(wheelBase / 2.0, -trackWidth / 2.0),
            new Translation2d(-wheelBase / 2.0, trackWidth / 2.0),
            new Translation2d(-wheelBase / 2.0, -trackWidth / 2.0));

        /* Module Gear Ratios */
        public static final double driveGearRatio = chosenModule.driveGearRatio;
        public static final double angleGearRatio = chosenModule.angleGearRatio;

        /* Motor Inverts */
        public static final boolean angleMotorInvert = chosenModule.angleMotorInvert;
        public static final boolean driveMotorInvert = chosenModule.driveMotorInvert;

        /* Angle Encoder Invert */
        public static final boolean canCoderInvert = chosenModule.canCoderInvert;

        /* Swerve Current Limiting */
        public static final int angleContinuousCurrentLimit = 25;
        public static final int anglePeakCurrentLimit = 40;
        public static final double anglePeakCurrentDuration = 0.1;
        public static final boolean angleEnableCurrentLimit = true;

        public static final int driveContinuousCurrentLimit = 35;
        public static final int drivePeakCurrentLimit = 60;
        public static final double drivePeakCurrentDuration = 0.1;
        public static final boolean driveEnableCurrentLimit = true;

        /* These values are used by the drive falcon to ramp in open loop and closed loop driving.
         * We found a small open loop ramp (0.25) helps with tread wear, tipping, etc */
        public static final double openLoopRamp = 0.25;
        public static final double closedLoopRamp = 0.0;

        /* Angle Motor PID Values */
        public static final double angleKP = chosenModule.angleKP;
        public static final double angleKI = chosenModule.angleKI;
        public static final double angleKD = chosenModule.angleKD;
        public static final double angleKF = chosenModule.angleKF;

        /* Drive Motor PID Values */
        public static final double driveKP = 1.6021; 
        public static final double driveKI = 0.0;
        public static final double driveKD = 0.02391;
        public static final double driveKF = 0.0;

        /* Drive Motor Characterization Values 
         * Divide SYSID values by 12 to convert from volts to percent output for CTRE */
        public static final double driveKS = (0.79074 / 12); 
        public static final double driveKV = (.194587 / 12);
        public static final double driveKA = (0.010932 / 12);

        /* Swerve Profiling Values */
        /** Meters per Second */
        public static final double maxSpeed = 4.5; //TODO: This must be tuned to specific robot
        /** Radians per Second */
        public static final double maxAngularVelocity = 10.0; //TODO: This must be tuned to specific robot

        /* Neutral Modes */
        public static final IdleMode angleNeutralMode = IdleMode.kCoast;
        public static final IdleMode driveNeutralMode = IdleMode.kBrake;
        
        /* Gear Ratio Stuff */
        
         public static final double angleRotationsToRadians = 360 /angleGearRatio; 
         public static final double angleRPMToRadiansPerSecond = angleRotationsToRadians / 60.0; 
         public static final double driveRotationsToMeters = wheelCircumference/driveGearRatio;
         public static final double driveRPMToMetersPerSecond = driveRotationsToMeters/60.0;
         

       


        public static final Translation2d[] kModuleTranslations = {
            new Translation2d(wheelBase / 2, trackWidth / 2),
            new Translation2d(wheelBase / 2, -trackWidth / 2),
            new Translation2d(-wheelBase / 2, trackWidth / 2),
            new Translation2d(-wheelBase / 2, -trackWidth / 2)
        };

        public enum ModulePosition {
            FRONT_LEFT,
            FRONT_RIGHT,
            BACK_LEFT,
            BACK_RIGHT
          };
        /* Module Specific Constants */
        /* Front Left Module - Module 0 */
        public static final class Mod0 { 
            public static final int driveMotorID = 6;
            public static final int angleMotorID = 5;
            public static final int canCoderID = 11;
            public static final Rotation2d angleOffset = Rotation2d.fromDegrees(138.779); 
            public static final SwerveModuleConstants constants = 
                new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset);
        }

        /* Front Right Module - Module 1 */
        public static final class Mod1 { 
            public static final int driveMotorID = 4;
            public static final int angleMotorID = 3;
            public static final int canCoderID = 12;
            public static final Rotation2d angleOffset = Rotation2d.fromDegrees(142.031);
            public static final SwerveModuleConstants constants = 
                new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset);
        }
        
        /* Back Left Module - Module 2 */
        public static final class Mod2 { 
            public static final int driveMotorID = 7;
            public static final int angleMotorID = 8;
            public static final int canCoderID = 13;
            public static final Rotation2d angleOffset = Rotation2d.fromDegrees(201.797);
            public static final SwerveModuleConstants constants = 
                new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset);
        }

        /* Back Right Module - Module 3 */
        public static final class Mod3 { 
            public static final int driveMotorID = 2;
            public static final int angleMotorID = 1;
            public static final int canCoderID = 14;
            public static final Rotation2d angleOffset = Rotation2d.fromDegrees(312.979);
            public static final SwerveModuleConstants constants = 
                new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset);
        }
    }

    public static final class AutoConstants { //TODO: The below constants are used in the example auto, and must be tuned to specific robot
        public static final double kMaxSpeedMetersPerSecond = SwerveConstants.maxSpeed;
        public static final double kMaxAccelerationMetersPerSecondSquared = 3;
        public static final double kMaxAngularSpeedRadiansPerSecond = SwerveConstants.maxAngularVelocity;
        public static final double kMaxAngularSpeedRadiansPerSecondSquared = Math.PI;
    
        public static final double kPXController = 1;
        public static final double kPYController = 1;
        public static final double kPThetaController = 1;
    
        /* Constraint for the motion profilied robot angle controller */
        public static final TrapezoidProfile.Constraints kThetaControllerConstraints =
            new TrapezoidProfile.Constraints(
                kMaxAngularSpeedRadiansPerSecond, kMaxAngularSpeedRadiansPerSecondSquared);

            };

    public static final class VisionConstants
    {

        public static final double alignToleranceMeters = Units.inchesToMeters(1);
        public static final double alignToleranceRadians = Units.degreesToRadians(1);

        public final static Transform3d robotToARCam = new Transform3d(new Translation3d(Units.inchesToMeters(0), Units.inchesToMeters(0), Units.inchesToMeters(0)), new Rotation3d(0,0,0));

        public final static double pi = Math.PI;
        public final static double nodeSpacing = Units.inchesToMeters(22);
        public final static double scoreDistance = Units.inchesToMeters(36); 

        //Red Alliance Align Spots
        public final static Pose2d pose1Right = new Pose2d(15.513558 - scoreDistance, 1.071626 - nodeSpacing, new Rotation2d(pi));
        public final static Pose2d pose1Mid = new Pose2d(15.513558 - scoreDistance, 1.071626, new Rotation2d(pi));
        public final static Pose2d pose1Left = new Pose2d(15.513558 - scoreDistance, 1.071626 + nodeSpacing, new Rotation2d(pi));
        public final static Pose2d pose2Right = new Pose2d(15.513558 - scoreDistance, 2.748026 - nodeSpacing, new Rotation2d(pi));
        public final static Pose2d pose2Mid = new Pose2d(15.513558 - scoreDistance, 2.748026, new Rotation2d(pi));
        public final static Pose2d pose2Left = new Pose2d(15.513558 - scoreDistance, 2.748026 + nodeSpacing, new Rotation2d(pi));
        public final static Pose2d pose3Right = new Pose2d(15.513558 - scoreDistance,4.424426 - nodeSpacing, new Rotation2d(pi));
        public final static Pose2d pose3Mid = new Pose2d(15.513558 - scoreDistance,4.424426, new Rotation2d(pi));
        public final static Pose2d pose3Left = new Pose2d(15.513558 - scoreDistance,4.424426 + nodeSpacing, new Rotation2d(pi));
        //Loading stations
        public final static Pose2d pose4 = new Pose2d(16.178784 - scoreDistance, 6.749796, new Rotation2d(pi));
        public final static Pose2d pose5 = new Pose2d(0.36195 + scoreDistance, 6.749796, new Rotation2d(0));
        //Blue Alliance align spots
        public final static Pose2d pose6Right = new Pose2d(1.02743 + scoreDistance, 4.424426 + nodeSpacing, new Rotation2d(0));
        public final static Pose2d pose6Mid = new Pose2d(1.02743 + scoreDistance, 4.424426, new Rotation2d(0));
        public final static Pose2d pose6Left = new Pose2d(1.02743 + scoreDistance, 4.424426 - nodeSpacing, new Rotation2d(0));
        public final static Pose2d pose7Right = new Pose2d(1.02743 + scoreDistance, 2.748026 + nodeSpacing, new Rotation2d(0));
        public final static Pose2d pose7Mid = new Pose2d(1.02743 + scoreDistance, 2.748026, new Rotation2d(0));
        public final static Pose2d pose7Left = new Pose2d(1.02743 + scoreDistance, 2.748026 - nodeSpacing, new Rotation2d(0));
        public final static Pose2d pose8Right = new Pose2d(1.02743 + scoreDistance, 1.071626 + nodeSpacing, new Rotation2d(0));
        public final static Pose2d pose8Mid = new Pose2d(1.02743 + scoreDistance, 1.071626, new Rotation2d(0));
        public final static Pose2d pose8Left = new Pose2d(1.02743 + scoreDistance, 1.071626 - nodeSpacing, new Rotation2d(0));

        public final static Pose2d[][] alignArray= 
        {{pose1Left, pose2Left, pose3Left, pose4, pose5, pose6Left, pose7Left, pose8Left},
        {pose1Mid, pose2Mid, pose3Mid, pose4,pose5,pose6Mid,pose7Mid,pose8Mid},
        {pose1Right,pose2Right, pose3Right,pose4, pose5, pose6Right, pose7Right, pose8Right}};
        }

        
}
  

