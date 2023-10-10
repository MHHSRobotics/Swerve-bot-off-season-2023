package frc.robot.Commands;

import edu.wpi.first.wpilibj2.command.*;
import frc.robot.Subsystems.Intake;

public class Intake_Commands {
    private final Intake intake;

    public Intake_Commands(Intake intake) {
        this.intake = intake;
    }

   public Command runIntake() {
        return new InstantCommand(() -> intake.intake(true));
   }

   public Command reverseIntake() {
        return new InstantCommand(() -> intake.intake(false));
   }

   public Command stopIntake() {
        return new InstantCommand(() -> intake.stop());
    }
}