package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.playingwithfusion.TimeOfFlight;
import com.playingwithfusion.TimeOfFlight.RangingMode;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ManipulatorSubsystem extends SubsystemBase {
    private final WPI_TalonFX motor;
    private final TimeOfFlight sensor;

    public ManipulatorSubsystem() {
        motor = new WPI_TalonFX(Constants.MANIPULATOR_MOTOR_PORT, Constants.CANIVORE_CAN_BUS_NAME);
        motor.setNeutralMode(NeutralMode.Brake);
        sensor = new TimeOfFlight(Constants.MANIPULATOR_DISTANCE_SENSOR);
        sensor.setRangingMode(RangingMode.Short, 40);
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
}
