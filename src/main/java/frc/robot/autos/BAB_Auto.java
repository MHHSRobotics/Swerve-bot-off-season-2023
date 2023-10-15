package frc.robot.Autos;

import frc.robot.Commands.*;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class BAB_Auto extends SequentialCommandGroup {

    public BAB_Auto(Swerve_Commands swerve, Elevator_Commands elevator, Intake_Commands intake) {
        addCommands(
            elevator.autoSetSpeed(0.5, 4),
            intake.autoIntake(false, 1)
            //swerve.autoMove(-1, -0.1, 0, 6)
            //swerve.autoBalance()
        );
    }
}
