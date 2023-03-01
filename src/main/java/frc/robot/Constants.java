package frc.robot;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
// import edu.wpi.first.wpilibj.DutyCycle;
import frc.robot.subsystems.arm.Setpoint;
import frc.robot.util.Gains;

public class Constants {
    public static final String CAN_BUS_NAME_CANIVORE = "FastFD";
    public static final String CAN_BUS_NAME_ROBORIO = "rio";
    public static final String CAN_BUS_NAME_DRIVETRAIN = CAN_BUS_NAME_CANIVORE;

    // IO Controller definitions
    public static final int CONTROLLER_USB_PORT = 0; // Drivers Controller
    public static final int GAMEPIECE_USB_PORT = 1; // Ordanence operators controller

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
    public static final int UPPER_ENCODER_ARM_CAN_ID = 25; // the CAN ID for the RIO CAN Bus
    public static final int LOWER_ENCODER_ARM_CAN_ID = 26; // the CAN ID for the RIO CAN Bus
    public static final int UPPER_JOINT_LEFT_MOTOR_CAN_ID = 5; // the CAN ID for the RIO CAN Bus
    public static final int UPPER_JOINT_RIGHT_MOTOR_CAN_ID = 6; // the CAN ID for the RIO CAN Bus
    public static final int LOWER_JOINT_LEFT_MOTOR_CAN_ID = 7; // the CAN ID for the RIO CAN Bus
    public static final int LOWER_JOINT_RIGHT_MOTOR_CAN_ID = 8; // the CAN ID for the RIO CAN Bus
    public static final int MANIPULATOR_WRIST_MOTOR_CAN_ID = 9;

    public static final class ArmConstants {

        public static final double LOWER_ANGLE_OFFSET = -244.4;
        public static final double SHOULDER_ANGLE_OFFSET = -89.4;

        public static final Gains GAINS_UPPER_JOINT = new Gains(0.02, 0.001, 0.01, 0.0, 50);

        public static final Gains GAINS_LOWER_JOINT = new Gains(0.02, 0.001, 0.01, 0.00, 50);

        public static final double kSUpper = 0.04;
        public static final double kGUpper = 0.25;
        public static final double kVUpper = 0.06;
        public static final double kAUpper = 0.0;

        public static final double kSLower = 0.04;
        public static final double kGLower = 0.25;
        public static final double kVLower = 0.06;
        public static final double kALower = 0.0;

        // JointConfig for DJArmFeedForwards
        public static final double UPPER_LENGTH = 1.07;
        public static final double UPPER_MOI = 0.4;
        public static final double UPPER_CGRADIUS = 1.0;
        public static final double UPPER_MASS = 5.0;
        public static final DCMotor UPPER_MOTOR = DCMotor.getFalcon500(2).withReduction(75);

        public static final double LOWER_LENGTH = 0.7874;
        public static final double LOWER_MOI = 0.4;
        public static final double LOWER_CGRADIUS = 1.0;
        public static final double LOWER_MASS = 5.0;
        public static final DCMotor LOWER_MOTOR = DCMotor.getFalcon500(2).withReduction(75);

        // Max sensor velocity per 100 ms
        // Max RPM 6380 * 2:1 gearing * 4096 ticks *1min/60000ms * 100ms
        public static final int MAX_SENSOR_VEL = 86398;

        /* Motor neutral dead-band : Range 0.001 -> 0.25 */
        public static final double NEUTRAL_DEADBAND = 0.005;

        public static final double NOMINAL_OUTPUT_FORWARD = 0;
        public static final double NOMINAL_OUTPUT_REVERSE = 0;
        public static final double PEAK_OUTPUT_FORWARD = 0.5;
        public static final double PEAK_OUTPUT_REVERSE = -0.5;

        public static final int FORWARD_SOFT_LIMIT_UPPER = 3300;
        public static final int REVERSE_SOFT_LIMIT_UPPER = 500;

        public static final int FORWARD_SOFT_LIMIT_LOWER = 3400;
        public static final int REVERSE_SOFT_LIMIT_LOWER = 1000;
        /**
         * Set to zero to skip waiting for confirmation.
         * Set to nonzero to wait and report to DS if action fails.
         */
        public final static int TIMEOUT = 10;

        // Motion Magic constants
        public static final double LOWER_CRUISE = 70.0;
        public static final double LOWER_ACCELERATION = 120.0;

        public static final double UPPER_CRUISE = 70.0;
        public static final double UPPER_ACCELERATION = 120.0;

        public static final double DUTY_CYCLE_MIN = 1.0 / 1025.0;
        public static final double DUTY_CYCLE_MAX = 1024.0 / 1025.0;
        public static final int FREQUENCY = 976;
        public static final double PERIOD = 1025;

        public static final double ENCODER_DISTANCE_PER_PULSE = (2.0 * Math.PI / 8192);
    }

    public static final class ArmSetpoints {
        public static final Setpoint TEST_SETPOINT_HIGHER = new Setpoint(191, 35, true, 191, 35, true);
        public static final Setpoint TEST_SETPOINT_LOWER = new Setpoint(164, 65, true, 164, 65, true);

        public static final Setpoint STOWED = new Setpoint(90, 90, false, 90, 90, false);
        public static final Setpoint FLOOR = new Setpoint(245, 61, true, 245, 62, true);
        public static final Setpoint MID_NODE = new Setpoint(192, 90, false, 192, 69, false);
        public static final Setpoint MID_NODE_PLACED = new Setpoint(199, 87, false, 192, 69, false);
        public static final Setpoint TOP_NODE = new Setpoint(225, 150, false, 213, 111, false);
        public static final Setpoint TOP_NODE_PLACED = new Setpoint(234, 149, false, 213, 111, false);
        public static final Setpoint SUBSTATION = new Setpoint(150, 53, false, 150, 53, false);

        public static final double INTERMEDIATE_LOWER_POSITION = 90;
    }

}
