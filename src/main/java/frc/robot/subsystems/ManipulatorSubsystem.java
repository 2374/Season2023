package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.playingwithfusion.TimeOfFlight;
import com.playingwithfusion.TimeOfFlight.RangingMode;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ManipulatorSubsystem extends SubsystemBase {
    private final WPI_TalonFX motor;
    TimeOfFlight sensor = new TimeOfFlight(18);

    public ManipulatorSubsystem() {
        motor = new WPI_TalonFX(Constants.MANIPULATOR_MOTOR_PORT, Constants.CANIVORE_CAN_BUS_NAME);
        motor.setNeutralMode(NeutralMode.Brake);
        // set the sensor to short range 1300 or less and sample evry 50ms
        sensor.setRangingMode(RangingMode.Short, 50);
        // restrict the image to the center of the sensor
        sensor.setRangeOfInterest(8, 8, 12, 12);
    }

    public void intake() {
        motor.set(.1);
    }

    public void outtake() {
        motor.set(-.1);
    }

    public void stop() {
        motor.stopMotor();
    }

    public double getDistance() {
        return sensor.getRange();
    }

    public void periodic() {
        // System.out.println("Sensor="+sensor.getRange());
    }
}
