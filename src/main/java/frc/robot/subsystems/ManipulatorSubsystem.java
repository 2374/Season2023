package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.playingwithfusion.TimeOfFlight;
import com.playingwithfusion.TimeOfFlight.RangingMode;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;

public class ManipulatorSubsystem extends SubsystemBase {
    private final WPI_TalonFX motor;
    TimeOfFlight sensor = new TimeOfFlight(Constants.MANIPULATOR_DISTANCE_SENSOR_CAN_ID);
    Boolean intakeMode = false; // we start the match with a cone in the manipulator ready to deploy
    RobotContainer robotContainer;

    public ManipulatorSubsystem(RobotContainer container) {
        motor = new WPI_TalonFX(Constants.MANIPULATOR_MOTOR_CAN_ID, Constants.CAN_BUS_NAME_ROBORIO);
        motor.setNeutralMode(NeutralMode.Brake);
        // set the sensor to short range 1300 or less and sample evry 50ms
        sensor.setRangingMode(RangingMode.Short, 50);
        // restrict the image to the center of the sensor
        sensor.setRangeOfInterest(8, 8, 12, 12);
        robotContainer = container; // give us a pointer back to the robot container to reference cube/cone desire
    }

    public void intake() {
        motor.set(.1);
        intakeMode = true;
    }

    public void outtake() {
        motor.set(-.1);
        intakeMode = false;
    }

    public void stop() {
        motor.stopMotor();
    }

    public double getDistance() {
        return sensor.getRange();
    }

    public void periodic() {
        // System.out.println("Sensor="+sensor.getRange());

        // The distance from the TOF Sensor comes back in cm from 10->1400ish
        // We want to stop the motor automatically when the intakeMode is true
        // at the correct distance for each object
        if (intakeMode) {
            if (robotContainer.getChassisSubsystem().getWantACone()) {
                if (sensor.getRange() < 100) {
                    stop();
                }
            } else { // we are testing for cube
                if (sensor.getRange() < 700) {
                    stop();
                }
            }
        }
    }
}
