package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.*;
import frc.robot.Subsystems.Intake;

public class Intake_Commands {
    private final Intake intake;

    public Intake_Commands(Intake intake) {
        this.intake = intake;
    }

   public Command runIntake() {
        return new InstantCommand(() -> {/* Put instructions here */});
   }

   public Command stopIntake() {
        return new InstantCommand(() -> {/* Put instructions here */});
    }
}