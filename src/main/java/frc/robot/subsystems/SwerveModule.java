package frc.robot.subsystems;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.util.CTREModuleState;
import frc.lib.util.SwerveModuleConstants;
import frc.robot.Constants;
import frc.robot.Robot;


import com.ctre.phoenix.sensors.CANCoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.REVPhysicsSim;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;


public class SwerveModule extends SubsystemBase{
    public int moduleNumber;
    private Rotation2d angleOffset;
    public Rotation2d lastAngle;

    private double m_simDriveEncoderPosition;
    private double m_simDriveEncoderVelocity;
    private double m_simAngleDifference;
    private double m_simTurnAngleIncrement;
    private double m_currentAngle;

    private CANSparkMax angleMotor;
    private CANSparkMax driveMotor;

    private RelativeEncoder driveEncoder;
    private RelativeEncoder integratedAngleEncoder;
    public CANCoder angleEncoder;

    private SparkMaxPIDController driveController;
    private SparkMaxPIDController angleController;

    private final SimpleMotorFeedforward feedforward = new SimpleMotorFeedforward(Constants.Swerve.driveKS, Constants.Swerve.driveKV, Constants.Swerve.driveKA);

    public SwerveModule(int moduleNumber, SwerveModuleConstants moduleConstants){
        this.moduleNumber = moduleNumber;
        this.angleOffset = moduleConstants.angleOffset;
        
        /* Angle Encoder Config */
        angleEncoder = new CANCoder(moduleConstants.cancoderID);
        configAngleEncoder();

        /* Angle Motor Config */
        angleMotor = new CANSparkMax(moduleConstants.angleMotorID, MotorType.kBrushless);
        integratedAngleEncoder = angleMotor.getEncoder();
        angleController = angleMotor.getPIDController();
        configAngleMotor();

        /* Drive Motor Config */
        driveMotor = new CANSparkMax(moduleConstants.driveMotorID, MotorType.kBrushless);
        driveEncoder = driveMotor.getEncoder();
        driveController = driveMotor.getPIDController();
        configDriveMotor();

        lastAngle = getState().angle;
        if (RobotBase.isSimulation()) {
            REVPhysicsSim.getInstance().addSparkMax(driveMotor, DCMotor.getNEO(1));
            REVPhysicsSim.getInstance().addSparkMax(angleMotor, DCMotor.getNEO(1));
            driveController.setP(1, 2);
          }
      
    }


    public void setDesiredState(SwerveModuleState desiredState, boolean isOpenLoop){
        desiredState = CTREModuleState.optimize(desiredState, getState().angle); 
        setAngle(desiredState);
        setSpeed(desiredState, isOpenLoop);
    }

    private void setSpeed(SwerveModuleState desiredState, boolean isOpenLoop){
        if(isOpenLoop){
            double percentOutput = desiredState.speedMetersPerSecond / Constants.Swerve.maxSpeed;
            driveMotor.set(percentOutput);
        }
        else {
            driveController.setReference(
                desiredState.speedMetersPerSecond,
                 ControlType.kVelocity, 0, 
                 feedforward.calculate(desiredState.speedMetersPerSecond));
        }
        
    }

    private void setAngle(SwerveModuleState desiredState){
        Rotation2d angle = (Math.abs(desiredState.speedMetersPerSecond) <= (Constants.Swerve.maxSpeed * 0.01)) ? lastAngle : desiredState.angle; //Prevent rotating module if speed is less then 1%. Prevents Jittering.
        angleController.setReference(angle.getDegrees(), ControlType.kPosition);
        lastAngle = angle;
        if (RobotBase.isSimulation()) {
            simUpdateDrivePosition(desiredState);
        //simTurnPosition(angle.getDegrees());
            m_currentAngle = angle.getDegrees();
          }
    }

    private Rotation2d getAngle(){
        
        
        if(RobotBase.isReal())
        {
            return Rotation2d.fromDegrees(integratedAngleEncoder.getPosition());
        }
        else 
        {
            return Rotation2d.fromDegrees(m_currentAngle);
        }
        
       
    }

    public Rotation2d getCanCoder(){
        return Rotation2d.fromDegrees(angleEncoder.getAbsolutePosition());
    }

    public void resetToAbsolute(){
        double absolutePosition = getCanCoder().getDegrees() - angleOffset.getDegrees();
        integratedAngleEncoder.setPosition(absolutePosition);
    }

    private void configAngleEncoder(){        
        angleEncoder.configFactoryDefault();
        angleEncoder.configAllSettings(Robot.ctreConfigs.swerveCanCoderConfig);
    }

    private void configAngleMotor(){
        angleMotor.restoreFactoryDefaults();
        angleMotor.setSmartCurrentLimit(Constants.Swerve.angleContinuousCurrentLimit);
        angleMotor.setInverted(Constants.Swerve.angleMotorInvert);
        angleMotor.setIdleMode(Constants.Swerve.angleNeutralMode);
        angleController.setP(Constants.Swerve.angleKP);
        angleController.setI(Constants.Swerve.angleKI);
        angleController.setD(Constants.Swerve.angleKD);
        angleController.setFF(Constants.Swerve.angleKF);
        angleController.setPositionPIDWrappingEnabled(true);
        angleController.setPositionPIDWrappingMaxInput(2 * Math.PI);
        angleController.setPositionPIDWrappingMinInput(0);
        resetToAbsolute();
        //TODO: Add this in REV HArdware client or here
        //angleEncoder.setPositionConversionFactor(Constants.kSwerve.ANGLE_ROTATIONS_TO_RADIANS);
        //angleEncoder.setVelocityConversionFactor(Constants.kSwerve.ANGLE_RPM_TO_RADIANS_PER_SECOND);
        //angleEncoder.setPosition(Units.degreesToRadians(canCoder.getAbsolutePosition() - canCoderOffsetDeg
    }

    private void configDriveMotor(){        
        driveMotor.restoreFactoryDefaults();
        driveMotor.setSmartCurrentLimit(Constants.Swerve.driveContinuousCurrentLimit);
        driveMotor.setInverted(Constants.Swerve.driveMotorInvert);
        driveMotor.setIdleMode(Constants.Swerve.driveNeutralMode);
        driveEncoder.setPosition(0);
        driveController.setP(Constants.Swerve.angleKP);
        driveController.setI(Constants.Swerve.angleKI);
        driveController.setD(Constants.Swerve.angleKD);
        driveController.setFF(Constants.Swerve.angleKF);
        driveMotor.burnFlash();
        driveEncoder.setPosition(0.0);

        //TODO: Add this in REV HArdware client or here
        //driveEncoder.setPositionConversionFactor(Constants.kSwerve.DRIVE_ROTATIONS_TO_METERS);
        //driveEncoder.setVelocityConversionFactor(Constants.kSwerve.DRIVE_RPM_TO_METERS_PER_SECOND);
        //driveEncoder.setPosition(0);
    }

    public double getVelocity()
    {
        if(RobotBase.isReal())
        return driveEncoder.getVelocity();
      else
        return m_simDriveEncoderVelocity;
    }

    public double getDrivePosition()
    {
        if(RobotBase.isReal())
      return driveEncoder.getPosition();
    else
      return m_simDriveEncoderPosition;
  
    }

    public SwerveModuleState getState(){
        return new SwerveModuleState(getVelocity(), getAngle()); 
    }

    public SwerveModulePosition getPosition(){
        return new SwerveModulePosition(getDrivePosition(), getAngle());
    }
    private void simUpdateDrivePosition(SwerveModuleState state) {
        m_simDriveEncoderVelocity = state.speedMetersPerSecond;
        double distancePer20Ms = m_simDriveEncoderVelocity / 50.0;
    
        m_simDriveEncoderPosition += distancePer20Ms;
      }
      private void simTurnPosition(double angle) {
        if (angle != lastAngle.getDegrees() && m_simTurnAngleIncrement == 0) {
          m_simAngleDifference = angle - lastAngle.getDegrees();
          m_simTurnAngleIncrement = m_simAngleDifference / 20.0;// 10*20ms = .2 sec move time
        }
        
}
@Override
  public void simulationPeriodic() {
    REVPhysicsSim.getInstance().run();
  }
}