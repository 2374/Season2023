package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
// import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.playingwithfusion.TimeOfFlight;
import com.playingwithfusion.TimeOfFlight.RangingMode;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;

public class ManipulatorSubsystem extends SubsystemBase {
    private final WPI_TalonFX intakeMotor;
    private final WPI_TalonFX wristRotationMotor;
    private TimeOfFlight sensor = new TimeOfFlight(Constants.MANIPULATOR_DISTANCE_SENSOR_CAN_ID);
    private Boolean intakeMode = false; // we start the match with a cone in the manipulator ready to deploy
    private Boolean active = false;
    private RobotContainer robotContainer;

    public ManipulatorSubsystem(RobotContainer container) {
        intakeMotor = new WPI_TalonFX(Constants.MANIPULATOR_MOTOR_CAN_ID, Constants.CAN_BUS_NAME_ROBORIO);
        intakeMotor.setNeutralMode(NeutralMode.Brake);
        wristRotationMotor = new WPI_TalonFX(Constants.MANIPULATOR_WRIST_MOTOR_CAN_ID, Constants.CAN_BUS_NAME_ROBORIO);
        // set the sensor to short range 1300 or less and sample evry 50ms
        sensor.setRangingMode(RangingMode.Short, 50);
        // restrict the image to the center of the sensor
        sensor.setRangeOfInterest(8, 8, 12, 12);
        robotContainer = container; // give us a pointer back to the robot container to reference cube/cone desire
    }

    public void intake() {
        intakeMotor.set(.4);
    }

    public void outtake() {
        intakeMotor.set(-.4);
    }

    public void stoptake() {
        intakeMotor.stopMotor();
    }

    public void rotateLeft() { // Use PID for this later
        wristRotationMotor.set(-.1);
    }

    public void rotateRight() {
        wristRotationMotor.set(.1);
    }

    public void stopRotation() {
        wristRotationMotor.stopMotor();
        wristRotationMotor.set(ControlMode.Position, 0);
    }

    public double getDistance() {
        return sensor.getRange();
    }

    public void activate() {
        active = true;
    }

    public void deactivate() {
        active = false;
    }

    public void periodic() {
        System.out.println("Sensor=" + robotContainer.getChassisSubsystem().getWantACone() + "  "
                + robotContainer.getChassisSubsystem().getWantACube() + "  " + sensor.getRange());

        // The distance from the TOF Sensor comes back in cm from 10->1400ish
        // We want to stop the motor automatically when the intakeMode is true
        // at the correct distance for each object
        if (active) {
            if (intakeMode) {
                if (robotContainer.getChassisSubsystem().getSomething()) {
                    if (robotContainer.getChassisSubsystem().getWantACone()) {
                        if (sensor.getRange() < 300) {
                            stoptake();
                            active = false;
                        }
                    } else if (robotContainer.getChassisSubsystem().getWantACube()) { // we are testing for cube
                        if (sensor.getRange() < 100) {
                            stoptake();
                            active = false;
                        }
                    } else {
                        stoptake();
                        active = false;
                    }
                }
            } else {
                if (robotContainer.getChassisSubsystem().getSomething()) {
                    if (sensor.getRange() > 300) {
                        stoptake();
                        active = false;
                    }
                }
            }
        }
    }
}
