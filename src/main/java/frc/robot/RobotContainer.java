package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

import frc.robot.autos.*;
import frc.robot.commands.*;
import frc.robot.subsystems.*;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
    /* Controllers */
    private final Joystick driver = new Joystick(0);


    /* Driver Buttons */
    private final JoystickButton robotCentric = new JoystickButton(driver, XboxController.Button.kLeftBumper.value);

    /* Subsystems */
    private final Swerve s_Swerve = new Swerve();

    private final FieldSim m_fieldSim = new FieldSim(s_Swerve);

    private final Auto m_auto = new Auto(s_Swerve);

    private final Vision vision = new Vision(); 



    /** The container for the robot. Contains subsystems, OI devices, and commands. */
    public RobotContainer() {
        s_Swerve.setDefaultCommand(
            new TeleopSwerve(
                s_Swerve, 
                () -> -driver.getRawAxis(1), 
                () -> -driver.getRawAxis(0), 
                () -> -driver.getRawAxis(4), 
                () -> robotCentric.getAsBoolean()
            ));

        vision.setDefaultCommand(new AddVisionMeasurement(s_Swerve, vision));
        

        // Configure the button bindings
        configureButtonBindings();
        m_fieldSim.initSim();

    }

    /**
     * Use this method to define your button->command mappings. Buttons can be created by
     * instantiating a {@link GenericHID} or one of its subclasses ({@link
     * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
     * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
     */
    private void configureButtonBindings() {
        /* Driver Buttons */
        new JoystickButton(driver, XboxController.Button.kY.value)
        .onTrue(new InstantCommand(() -> s_Swerve.zeroGyro()));

        new JoystickButton(driver, XboxController.Button.kX.value)
        .whileTrue(new AlignRobot(s_Swerve, vision, "leftCone"));

        new JoystickButton(driver, XboxController.Button.kA.value)
        .whileTrue(new AlignRobot(s_Swerve, vision, "midCube"));
        
        new JoystickButton(driver, XboxController.Button.kB.value)
        .whileTrue(new AlignRobot(s_Swerve, vision, "rightCone"));
    }

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {
        // An ExampleCommand will run in autonomous
        return m_auto.getAuto();


        
    }
    
    public void periodic() {
        m_fieldSim.periodic();
      }
}
