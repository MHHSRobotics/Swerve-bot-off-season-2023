package frc.robot.Subsystems;

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
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

/* Swerve Module Subsystem that controls individual modules. Created by FRC 364 and modified by FRC 5137. */
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

    private final SimpleMotorFeedforward feedforward = new SimpleMotorFeedforward(Constants.SwerveConstants.driveKS, Constants.SwerveConstants.driveKV, Constants.SwerveConstants.driveKA);

    /**
     * Object for controlling each individual swerve module.
     * @param moduleNumber the ID assigned to each swerveModule(0-3). Check Constants for explanation of each number. 
     * @param moduleConstants the natural angle offset of the module, when facing straight, according to the absolute encoder. 
     */
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
       
       /*For simulation only */
        if (RobotBase.isSimulation()) {
            REVPhysicsSim.getInstance().addSparkMax(driveMotor, DCMotor.getNEO(1));
            REVPhysicsSim.getInstance().addSparkMax(angleMotor, DCMotor.getNEO(1));
            driveController.setP(1, 2);
          }
    }


    /**
     * Sets the angle motors and drive motors to move based on desired
     * movement from Swerve Drive Subsystem. Optimizes turning to avoid overotation. 
     * Consult Joaquin if confused. 
     * @param desiredState how the swerve subsystem wants the specific module to move. 
     * @param isOpenLoop if the directional speed should use feedback control. 
     */
    public void setDesiredState(SwerveModuleState desiredState, boolean isOpenLoop){
        desiredState = CTREModuleState.optimize(desiredState, getState().angle); 
        setAngle(desiredState);
        setSpeed(desiredState, isOpenLoop);
    }

    /**
     * Last step of the process used to control the individual modules drive speed directly. 
     * @param desiredState a paramater representing the desired module movement.
     * @param isOpenLoop decideds if it should be feedback or non-feedback controlled. 
     * If true:
     * determines a percent output based upon desiredState (typically for TeleOp). 
     * If false:
     * uses velocity feedback control to move precisely (typically for Auto).
     */
    private void setSpeed(SwerveModuleState desiredState, boolean isOpenLoop){
        if(isOpenLoop){
            double percentOutput = desiredState.speedMetersPerSecond / Constants.SwerveConstants.maxSpeed;
            driveMotor.set(percentOutput);
        }
        else {
            driveController.setReference(
                desiredState.speedMetersPerSecond,
                 ControlType.kVelocity, 0, 
                 feedforward.calculate(desiredState.speedMetersPerSecond));
        }
        
    }

    /**
     * Tells the angleController to move to a specific angle using position control. 
     * @param desiredState the desired movement of the specific module. 
     */
    private void setAngle(SwerveModuleState desiredState){
        Rotation2d angle = (Math.abs(desiredState.speedMetersPerSecond) <= (Constants.SwerveConstants.maxSpeed * 0.01)) ? lastAngle : desiredState.angle; //Prevent rotating module if speed is less then 1%. Prevents Jittering.
        angleController.setReference(angle.getDegrees(), ControlType.kPosition);
        lastAngle = angle;
        if (RobotBase.isSimulation()) {
            simUpdateDrivePosition(desiredState);
        //simTurnPosition(angle.getDegrees());
            m_currentAngle = angle.getDegrees();
          }
    }


    /**
     * Gets the angle of the module wheel. 
     * @return the current angle of the module wheel in degrees. 
     */
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

    /**
     * Gets the angle of the module from the absolute encoder.
     * @return absolute encoder angle in degrees.
     */
    public Rotation2d getCanCoder(){
        return Rotation2d.fromDegrees(angleEncoder.getAbsolutePosition());
    }

    /**
     * Makes sure that angle controller contains current CAN Coder 
     * reading. 
     */
    public void resetToAbsolute(){
        double absolutePosition = getCanCoder().getDegrees() - angleOffset.getDegrees();
        System.out.println("absolute" + absolutePosition);  
        integratedAngleEncoder.setPosition(absolutePosition);
    }

    private void configAngleEncoder(){      
        angleEncoder.configFactoryDefault();
        angleEncoder.configAllSettings(Robot.ctreConfigs.swerveCanCoderConfig);
    }

    private void configAngleMotor(){
        angleMotor.restoreFactoryDefaults();
        angleMotor.setSmartCurrentLimit(Constants.SwerveConstants.angleContinuousCurrentLimit);
        angleMotor.setInverted(Constants.SwerveConstants.angleMotorInvert);
        angleMotor.setIdleMode(Constants.SwerveConstants.angleNeutralMode);
        angleController.setP(Constants.SwerveConstants.angleKP*(2*Math.PI)/360);
        angleController.setI(Constants.SwerveConstants.angleKI*(2*Math.PI)/360);
        angleController.setD(Constants.SwerveConstants.angleKD*(2*Math.PI)/360);
        angleController.setFF(Constants.SwerveConstants.angleKF*(2*Math.PI)/360);
        angleController.setPositionPIDWrappingEnabled(true);
        angleController.setPositionPIDWrappingMaxInput(360);
        angleController.setPositionPIDWrappingMinInput(1);
        resetToAbsolute();
        integratedAngleEncoder.setPositionConversionFactor(Constants.SwerveConstants.angleRotationsToRadians);
        integratedAngleEncoder.setVelocityConversionFactor(Constants.SwerveConstants.angleRPMToRadiansPerSecond);
        angleMotor.burnFlash();
    }

    private void configDriveMotor(){        
        driveMotor.restoreFactoryDefaults();
        driveMotor.setSmartCurrentLimit(Constants.SwerveConstants.driveContinuousCurrentLimit);
        driveMotor.setInverted(Constants.SwerveConstants.driveMotorInvert);
        driveMotor.setIdleMode(Constants.SwerveConstants.driveNeutralMode);
        driveEncoder.setPosition(0);
        driveController.setP(Constants.SwerveConstants.angleKP);
        driveController.setI(Constants.SwerveConstants.angleKI);
        driveController.setD(Constants.SwerveConstants.angleKD);
        driveController.setFF(Constants.SwerveConstants.angleKF);
        driveEncoder.setPosition(0.0);
        driveEncoder.setPositionConversionFactor(Constants.SwerveConstants.driveRotationsToMeters);
        driveEncoder.setVelocityConversionFactor(Constants.SwerveConstants.driveRPMToMetersPerSecond);
        driveEncoder.setPosition(0); 
        driveMotor.burnFlash();
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
public void periodic()
{
    System.out.println("speed" +driveMotor.get());
    
}
@Override
  public void simulationPeriodic() {
    REVPhysicsSim.getInstance().run();
  }
}