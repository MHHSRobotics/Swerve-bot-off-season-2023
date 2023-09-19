package frc.robot.Subsystems;

import frc.robot.Constants;

import java.sql.Driver;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Elevator extends SubsystemBase {
    private Joystick driver;
    private static Spark motor;
    private static Encoder encoder;
    private static PIDController PID;
    private static TrapezoidProfile TP;
    private static TrapezoidProfile.Constraints TP_Constraints;
    private static double goal;
    private static double position;
    private static double kV;

    public Elevator(Joystick driver) {
        this.driver = driver;
        motor = new Spark(Constants.ElevatorConstants.motorPort);
        encoder = new Encoder(0, 1);
        PID = new PIDController(0, 0, 0);
        TP_Constraints = new TrapezoidProfile.Constraints(Constants.ElevatorConstants.maxVelo, Constants.ElevatorConstants.maxAccel);
    }

    public void periodic() {
        position = encoder.getDistance();
        if (Math.abs(driver.getRawAxis(1)) > 0.2) {
            kV = driver.getRawAxis(1);
        } else { 
            //var target = TP.calculate(0.02);
            //kV = PID.calculate(position, goal);
        }
        motor.setVoltage(kV);
    }

    public void set(double x) {
        goal = x;
        //TP = new TrapezoidProfile(TP_Constraints, new TrapezoidProfile.State(goal, 0.0), new TrapezoidProfile.State(position, kV));
    }

    public double get() {
        return position;
    }
}
