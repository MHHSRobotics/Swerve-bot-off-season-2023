package frc.robot.Subsystems;

import frc.robot.Constants;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

public class Elevator extends SubsystemBase {
    private Joystick controller;
    private static CANSparkMax motor;
    private static Encoder encoder;
    private static PIDController PID;
    private static TrapezoidProfile TP;
    private static TrapezoidProfile.Constraints TP_Constraints;
    private static double goal;
    private static double position;
    private static double kV;

    public Elevator(Joystick controller) {
        this.controller = controller;
        motor = new CANSparkMax(Constants.ElevatorConstants.motorPort, MotorType.kBrushed);
        encoder = new Encoder(0, 1);
        PID = new PIDController(0, 0, 0);
        TP_Constraints = new TrapezoidProfile.Constraints(Constants.ElevatorConstants.maxVelo, Constants.ElevatorConstants.maxAccel);
    }

    public void periodic() {
        position = encoder.getDistance();
        if (Math.abs(controller.getRawAxis(1)) > 0.2) {
            kV = -controller.getRawAxis(1);
            goal = position;
        } else {
            kV = 0.0;
            //var target = TP.calculate(0.02);
            //kV = PID.calculate(position, goal);
        }
        motor.setVoltage(kV);
        System.out.println("Position: "+position);
        System.out.println("Goal: "+goal);
    }

    public void set(double x) {
        goal = x;
        //TP = new TrapezoidProfile(TP_Constraints, new TrapezoidProfile.State(goal, 0.0), new TrapezoidProfile.State(position, kV));
    }

    public double get() {
        return position;
    }
}
