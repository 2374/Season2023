package frc.robot;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import frc.common.control.PidConstants;
// import edu.wpi.first.wpilibj.DutyCycle;
import frc.robot.subsystems.arm.Setpoint;
import frc.robot.util.Gains;

public class Constants {
    public static final String CAN_BUS_NAME_CANIVORE = "FastFD";
    public static final String CAN_BUS_NAME_ROBORIO = "rio";
    public static final String CAN_BUS_NAME_DRIVETRAIN = CAN_BUS_NAME_CANIVORE;

    // IO Controller definitions
    public static final int CONTROLLER_USB_PORT_DRIVER = 0; // Drivers Controller
    public static final int CONTROLLER_USB_PORT_OPERATOR = 1; // Ordanence operators controller

    // ARM Subsystem
    public static final int ARM_MOTOR_CAN_ID = 8; // the CAN ID for the RIO CAN Bus
    public static final double ARM_LIMIT = 0;

    // DRIVETRAIN Subsystem
    public static final double DRIVETRAIN_LENGTH_METERS = Units.inchesToMeters(20.5);
    public static final double DRIVETRAIN_WIDTH_METERS = Units.inchesToMeters(20.5);
    public static final int DRIVETRAIN_PIGEON_CAN_ID = 29; // the CAN ID for the FASTFD CAN Bus
    // Front Left Swerve Module
    public static final int FRONT_LEFT_MODULE_DRIVE_MOTOR_CAN_ID = 1; // the CAN ID for the FASTFD CAN Bus
    public static final int FRONT_LEFT_MODULE_STEER_MOTOR_CAN_ID = 11; // the CAN ID for the FASTFD CAN Bus
    public static final int FRONT_LEFT_MODULE_STEER_ENCODER_CAN_ID = 21; // the CAN ID for the FASTFD CAN Bus
    public static final double FRONT_LEFT_MODULE_STEER_OFFSET_TEST = -Math.toRadians(190.27); // Swervee Module Offset
    public static final double FRONT_LEFT_MODULE_STEER_OFFSET = -Math.toRadians(182.19 + 180); // Comp Module Offset
    // Front Right Swerve Module
    public static final int FRONT_RIGHT_MODULE_DRIVE_MOTOR_CAN_ID = 2; // the CAN ID for the FASTFD CAN Bus
    public static final int FRONT_RIGHT_MODULE_STEER_MOTOR_CAN_ID = 12; // the CAN ID for the FASTFD CAN Bus
    public static final int FRONT_RIGHT_MODULE_STEER_ENCODER_CAN_ID = 22; // the CAN ID for the FASTFD CAN Bus
    public static final double FRONT_RIGHT_MODULE_STEER_OFFSET_TEST = -Math.toRadians(261.88); // Swervee Module Offset
    public static final double FRONT_RIGHT_MODULE_STEER_OFFSET = -Math.toRadians(214.1 + 180); // Comp Module Offset
    // Back Left Swerve Module
    public static final int BACK_LEFT_MODULE_DRIVE_MOTOR_CAN_ID = 3; // the CAN ID for the FASTFD CAN Bus
    public static final int BACK_LEFT_MODULE_STEER_MOTOR_CAN_ID = 13; // the CAN ID for the FASTFD CAN Bus
    public static final int BACK_LEFT_MODULE_STEER_ENCODER_CAN_ID = 23; // the CAN ID for the FASTFD CAN Bus
    public static final double BACK_LEFT_MODULE_STEER_OFFSET_TEST = -Math.toRadians(126.02); // Swervee Module Offset
    public static final double BACK_LEFT_MODULE_STEER_OFFSET = -Math.toRadians(57.83); // Comp Module Offset
    // Back Right Swerve Module
    public static final int BACK_RIGHT_MODULE_DRIVE_MOTOR_CAN_ID = 4; // the CAN ID for the FASTFD CAN Bus
    public static final int BACK_RIGHT_MODULE_STEER_MOTOR_CAN_ID = 14; // the CAN ID for the FASTFD CAN Bus
    public static final int BACK_RIGHT_MODULE_STEER_ENCODER_CAN_ID = 24; // the CAN ID for the FASTFD CAN Bus
    public static final double BACK_RIGHT_MODULE_STEER_OFFSET_TEST = -Math.toRadians(263.67); // Swervee Module Offset
    public static final double BACK_RIGHT_MODULE_STEER_OFFSET = -Math.toRadians(154.41 + 180); // Comp Module Offset

    // SHOOTER Subsystem
    public static final int HOOD_MOTOR_CAN_ID = 0; // 14;
    public static final int FLYWHEEL_PRIMARY_MOTOR_CAN_ID = 0; // 15;
    public static final int FLYWHEEL_SECONDARY_MOTOR_CAN_ID = 0; // 9;
    public static final double HOOD_MANUAL_ADJUST_INTERVAL = Math.toRadians(0.5);
    public static final double FLYWHEEL_MANUAL_ADJUST_INTERVAL = Units.rotationsPerMinuteToRadiansPerSecond(25.0);
    public static final double HOOD_MOTOR_TO_HOOD_GEAR_RATIO = 1;
    public static final double HOOD_SHOOTING_ALLOWABLE_ERROR = Math.toRadians(0.5);
    public static final double HOOD_CLIMBING_ALLOWABLE_ERROR = Math.toRadians(1.0);
    public static final String SHOOTER_OFFSET_ENTRY_NAME = "Shooting Offset";
    public static final String HOOD_OFFSET_ENTRY_NAME = "Hood Offset";
    public static final String DRIVER_READOUT_TAB_NAME = "Driver Readout";
    public static final int CLAW_MOTOR_CAN_ID = 17;

    // CHASSIS Subsystem
    public static final int CANDLE_CAN_ID = 27; // the CAN ID for the FASTFD CAN Bus
    public static final String TEST_ROBORIO_SERIAL_NUMBER = "0316b2d6"; // serial number of Swervee roborio

    // MANIPULATOR Subsystem
    public static final int MANIPULATOR_MOTOR_CAN_ID = 10; // the CAN ID for the RIO CAN Bus
    public static final int MANIPULATOR_DISTANCE_SENSOR_CAN_ID = 18; // the CAN ID for the RIO CAN Bus

    // Field measurements
    public static final double FIELD_LENGTH = Units.feetToMeters(54);
    public static final double FIELD_WIDTH = Units.feetToMeters(27);

    // Vision Stuff
    public static final String CAMERA_NAME = "photonvision";
    public static final Transform3d ROBOT_TO_CAMERA = new Transform3d(new Translation3d(.3, 0, 0.2),
            new Rotation3d(0, 0, 0));
    public static final Pose3d TAG_1_POSE3D = new Pose3d(FIELD_LENGTH, FIELD_WIDTH / 2, 4.5, new Rotation3d(0, 0, 180));
    public static final boolean TEST_MODE = false;
    public static final int SHOULDER_ENCODER_ARM_CAN_ID = 25; // the CAN ID for the RIO CAN Bus
    public static final int ELBOW_ENCODER_ARM_CAN_ID = 26; // the CAN ID for the RIO CAN Bus
    public static final int SHOULDER_JOINT_LEFT_MOTOR_CAN_ID = 5; // the CAN ID for the RIO CAN Bus
    public static final int SHOULDER_JOINT_RIGHT_MOTOR_CAN_ID = 6; // the CAN ID for the RIO CAN Bus
    public static final int ELBOW_JOINT_LEFT_MOTOR_CAN_ID = 7; // the CAN ID for the RIO CAN Bus
    public static final int ELBOW_JOINT_RIGHT_MOTOR_CAN_ID = 8; // the CAN ID for the RIO CAN Bus
    public static final int MANIPULATOR_WRIST_MOTOR_CAN_ID = 9;
    public static final double WRIST_OFFSET = -68.3789;
    public static final PidConstants WRIST_PID_CONSTANTS = new PidConstants(0.00002, 0, 0);

    public static final class ArmConstants {

        public static final double ELBOW_ANGLE_OFFSET = -5.27; // -36.29; //-244.4;
        public static final double SHOULDER_ANGLE_OFFSET = -27.7;

        // public static final Gains GAINS_SHOULDER_JOINT = new Gains(0.9, 0.1, 0.01,
        // 0.0, 50);

        // public static final Gains GAINS_ELBOW_JOINT = new Gains(0.9, 0.1, 0.01, 0.00,
        // 50);

        public static final double kSShoulder = 0.04;
        public static final double kGShoulder = 0.25;
        public static final double kVShoulder = 0.06;
        public static final double kAShoulder = 0.0;

        public static final double kSElbow = 0.04;
        public static final double kGElbow = 0.25;
        public static final double kVElbow = 0.06;
        public static final double kAElbow = 0.0;

        // JointConfig for DJArmFeedForwards
        public static final double SHOULDER_LENGTH = 1.07;
        public static final double SHOULDER_MOI = 0.4;
        public static final double SHOULDER_CGRADIUS = 1.0;
        public static final double SHOULDER_MASS = 5.0;
        public static final DCMotor SHOULDER_MOTOR = DCMotor.getFalcon500(2).withReduction(80);

        public static final double ELBOW_LENGTH = 0.7874;
        public static final double ELBOW_MOI = 0.4;
        public static final double ELBOW_CGRADIUS = 1.0;
        public static final double ELBOW_MASS = 5.0;
        public static final DCMotor ELBOW_MOTOR = DCMotor.getFalcon500(2).withReduction(60);

        // Max sensor velocity per 100 ms
        // Max RPM 6380 * 2:1 gearing * 4096 ticks *1min/60000ms * 100ms
        public static final int MAX_SENSOR_VEL = 86398;

        /* Motor neutral dead-band : Range 0.001 -> 0.25 */
        public static final double NEUTRAL_DEADBAND = 0.01;

        public static final double NOMINAL_OUTPUT_FORWARD = 0;
        public static final double NOMINAL_OUTPUT_REVERSE = 0;
        public static final double PEAK_OUTPUT_FORWARD = 0.5;
        public static final double PEAK_OUTPUT_REVERSE = -0.5;

        public static final int FORWARD_SOFT_LIMIT_SHOULDER = 3300;
        public static final int REVERSE_SOFT_LIMIT_SHOULDER = 500;

        public static final int FORWARD_SOFT_LIMIT_ELBOW = 3400;
        public static final int REVERSE_SOFT_LIMIT_ELBOW = 1000;
        /**
         * Set to zero to skip waiting for confirmation.
         * Set to nonzero to wait and report to DS if action fails.
         */
        public final static int TIMEOUT = 10;

        // Motion Magic constants
        public static final double ELBOW_CRUISE = 35.0;
        public static final double ELBOW_ACCELERATION = 60.0;

        public static final double SHOULDER_CRUISE = 35.0;
        public static final double SHOULDER_ACCELERATION = 60.0;

        public static final double DUTY_CYCLE_MIN = 1.0 / 1025.0;
        public static final double DUTY_CYCLE_MAX = 1024.0 / 1025.0;
        public static final int FREQUENCY = 976;
        public static final double PERIOD = 1025;

        public static final double ENCODER_DISTANCE_PER_PULSE = (2.0 * Math.PI / 8192);
        public static final Gains GAINS_SHOULDER_JOINT = new Gains(0.8, 0.3, 0.0, 0.0, 50);

        // public static final Gains GAINS_ELBOW_JOINT = new Gains(0.75, 0.75, 0.0,
        // 0.00, 50);
        public static final Gains GAINS_ELBOW_JOINT = new Gains(0.4, 0.3, 0.0, 0.00, 50);
    }

    public static final class ArmSetpoints {
        public static final Setpoint TEST_SETPOINT_HIGHER = new Setpoint(0, 191, 35, true, 191, 35, true);
        public static final Setpoint TEST_SETPOINT_LOWER = new Setpoint(0, 164, 65, true, 164, 65, true);

        // ELBOW | SHOULDER false/true
        public static final Setpoint TALL = new Setpoint(0, 1, 1, false, 1, 1, false);
        public static final Setpoint REST = new Setpoint(REST_Label, -140, 132, false, -140, 132, false);
        public static final Setpoint MID_READY = new Setpoint(MID_READY_Label, 35, 60, false, 35, 60, false);
        public static final Setpoint MID_DROP = new Setpoint(MID_DROP_Label, 30, 50, false, 30, 50, false);
        public static final Setpoint STOW = new Setpoint(STOW_Label, 140, 60, false, 140, 60, false);
        public static final Setpoint STAB_READY = new Setpoint(STAB_READY_Label, 90, 60, false, 90, 60, false);
        public static final Setpoint STAB = new Setpoint(STAB_Label, 62, 95, false, 62, 95, false);
        public static final Setpoint LOW_SCORE = new Setpoint(LOW_SCORE_Label, 90, 60, false, 90, 60, false);
        public static final Setpoint HIGH_SCORE = new Setpoint(HIGH_SCORE_Label, -35, 75, false, -35, 75, false);

        public static final double INTERMEDIATE_ELBOW_POSITION = 90;
    }

    public static final int REST_Label = 1;
    public static final int MID_READY_Label = 2;
    public static final int MID_DROP_Label = 3;
    public static final int STOW_Label = 4;
    public static final int STAB_READY_Label = 5;
    public static final int STAB_Label = 6;
    public static final int LOW_SCORE_Label = 7;
    public static final int HIGH_SCORE_Label = 8;

}
