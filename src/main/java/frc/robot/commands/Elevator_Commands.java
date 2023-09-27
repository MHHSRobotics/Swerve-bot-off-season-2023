package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.*;
import frc.robot.Subsystems.Elevator;

public class Elevator_Commands {
    private final Elevator elevator;

    public Elevator_Commands(Elevator elevator) {
        this.elevator = elevator;
    }

   public Command setPosition(double pos) {
        return new InstantCommand(() -> elevator.set(pos));
   } 
}