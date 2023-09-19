package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

import frc.robot.autos.*;
import frc.robot.commands.*;
import frc.robot.Subsystems.*;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
    /* Controllers */
    private final Joystick driver = new Joystick(0);
    private final Joystick assist = new Joystick(1);
    //private final Joystick troll = new Joystick(2);

    /* Driver Buttons */
    private final JoystickButton robotCentric = new JoystickButton(driver, XboxController.Button.kLeftBumper.value);

    /* Subsystems */
    private final Swerve s_Swerve = new Swerve();

    private final FieldSim m_fieldSim = new FieldSim(s_Swerve);

    private final Auto m_auto = new Auto(s_Swerve);

    private final Vision vision = new Vision(); 

    private final Elevator elevator_Subsystem = new Elevator(driver);

    /* Commands */

    private final Elevator_Commands elevator = new Elevator_Commands(elevator_Subsystem);

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

    private void configureButtonBindings() {
        /* Driver Buttons */
        new JoystickButton(driver, XboxController.Button.kY.value)
        .onTrue(new InstantCommand(() -> s_Swerve.zeroGyro()));

        new JoystickButton(driver, XboxController.Button.kX.value)
        .whileTrue(new AlignVision(s_Swerve, vision, "leftCone"));

        new JoystickButton(driver, XboxController.Button.kA.value)
        .whileTrue(new AlignVision(s_Swerve, vision, "midCube"));
        
        new JoystickButton(driver, XboxController.Button.kB.value)
        .whileTrue(new AlignVision(s_Swerve, vision, "rightCone"));

        /* Operator Buttons */

        new JoystickButton(assist, XboxController.Button.kY.value)
        .onTrue(elevator.setPosition(0));

        new JoystickButton(assist, XboxController.Button.kX.value)
        .onTrue(elevator.setPosition(1));

        new JoystickButton(assist, XboxController.Button.kA.value)
        .onTrue(elevator.setPosition(2));

        new JoystickButton(assist, XboxController.Button.kB.value)
        .onTrue(elevator.setPosition(3));
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
