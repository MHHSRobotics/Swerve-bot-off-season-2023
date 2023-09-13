package frc.robot.Subsystems;

import frc.robot.Constants;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Elevator extends SubsystemBase {
    private static Spark motor;
    private static Encoder encoder;
    private static PIDController PID;
    private static TrapezoidProfile TP;
    private static TrapezoidProfile.Constraints TP_Constraints;
    private static double goal;
    private static double position;
    private static double velo;

    public Elevator() {
        motor = new Spark(Constants.ElevatorConstants.motorPort);
        encoder = new Encoder(0, 1);
        PID = new PIDController(0, 0, 0);
        TP_Constraints = new TrapezoidProfile.Constraints(Constants.ElevatorConstants.maxVelo, Constants.ElevatorConstants.maxAccel);
    }

    public void periodic() {
        position = encoder.getDistance();
        var target = TP.calculate(0.02);
        velo = PID.calculate(position, target.position);
        motor.setVoltage(velo);
    }

    public void set(double x) {
        goal = x;
        TP = new TrapezoidProfile(TP_Constraints, new TrapezoidProfile.State(goal, 0.0), new TrapezoidProfile.State(position, velo));
    }

    public double get() {
        return position;
    }
}
