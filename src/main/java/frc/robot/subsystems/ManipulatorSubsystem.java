package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
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
    private Boolean active = false; // are we actively intaking or outtaking?
    private RobotContainer robotContainer;
    private int foundCounter = 0;  

    /**
     * The subsystem for managing the manipulator and the wrist
     * 
     * @param container The robot container
     */
    public ManipulatorSubsystem(RobotContainer container) {
        intakeMotor = new WPI_TalonFX(Constants.MANIPULATOR_MOTOR_CAN_ID, Constants.CAN_BUS_NAME_ROBORIO);
        intakeMotor.setNeutralMode(NeutralMode.Brake);
        intakeMotor.setInverted(true);
        wristRotationMotor = new WPI_TalonFX(Constants.MANIPULATOR_WRIST_MOTOR_CAN_ID, Constants.CAN_BUS_NAME_ROBORIO);
        // set the sensor to short range 1300 or less and sample evry 200ms
        sensor.setRangingMode(RangingMode.Short, 50);
        // restrict the image to the center of the sensor 16x16 is the full grid
        sensor.setRangeOfInterest(7, 7, 9, 9);
        robotContainer = container; // give us a pointer back to the robot container to reference cube/cone desire
        
    }

    /**
     * Starts intaking
     */
    public void intake() {
        intakeMotor.set(1);
        intakeMode=true;
        foundCounter = 0;
        activate();
    }

    /**
     * Starts outtaking
     */
    public void outtake() {
        intakeMotor.set(-1);
        intakeMode=false;
        activate();
    }

    /**
     * Stops taking
     */
    public void stoptake() {
        intakeMotor.stopMotor();
        deactivate();
    }

    /**
     * rotate wrist left
     */
    public void rotateLeft() { // Use PID for this later
        wristRotationMotor.set(-.1);
    }

    /**
     * rotate wrist right
     */
    public void rotateRight() {
        wristRotationMotor.set(.1);
    }

    /**
     * stops rotating wrist
     */
    public void stopRotation() {
        wristRotationMotor.stopMotor();
    }

    /**
     * @return The distance measured by the Time of Flight Sensor in millimeters
     */
    public double getDistance() {
        return sensor.getRange();
    }

    /**
     * Activates the periodic sensing
     */
    private void activate() {
        active = true;
    }

    /**
     * deactivates the periodic sensing
     */
    private void deactivate() {
        active = false;
    }

    public void periodic() {
        // The distance from the TOF Sensor comes back in cm from 10->1400ish
        // We want to stop the motor automatically when the intakeMode is true
        // at the correct distance for each object
        if (active) { // is active?
            System.out.println("Sensor=" + robotContainer.getChassisSubsystem().getWantACone() + " " + robotContainer.getChassisSubsystem().getWantACube() + " " + sensor.getRange());

            if (intakeMode) { // intake mode?
                if (robotContainer.getChassisSubsystem().getSomething()) { // want something?
                    if (robotContainer.getChassisSubsystem().getWantACone()) { // want cone?
                        if (sensor.getRange() < 60) {
                            foundCounter++;
                            if (foundCounter > 10) {
                                System.out.println("stopping due to enough results for cube less than 60");
                                stoptake();
                            }
                        } else {
                            foundCounter = 0;
                        }
                    } else if (robotContainer.getChassisSubsystem().getWantACube()) { // want cube?
                        
                        if (sensor.getRange() < 130) {
                            foundCounter++;
                            if (foundCounter > 15) {
                                System.out.println("stopping due to enough results for cube less than 130");
                                stoptake();
                            }
                        } else {
                            foundCounter = 0;
                        }
                    } else {
                        System.out.println("stopping due to invalid state");
                        stoptake();
                    }
                }
            } else { // outtake mode
                if (sensor.getRange() > 300) {
                    foundCounter--;
                    if (foundCounter <= 0 ) {
                        System.out.println("stopping due to game piece having left long enough");
                        stoptake();
                    }
                }
            }
        }
    }
}
