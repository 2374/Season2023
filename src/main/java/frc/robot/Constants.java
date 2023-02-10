package frc.robot;


import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;

public class Constants {
    public static final String CANIVORE_CAN_BUS_NAME = "FastFD";
    public static final String RIO_CAN_BUS_NAME = "rio";
    public static final String DRIVETRAIN_CAN_BUS_NAME = CANIVORE_CAN_BUS_NAME;

    // IO Controller definitions
    public static final int CONTROLLER_PORT = 0;  // Drivers Controller
    public static final int GAMEPIECE_PORT  = 1;  // Ordanence operators controller

    // ARM Subsystem
    public static final int ARM_MOTOR_PORT = 8;
    public static final double ARM_LIMIT = 0;

    public static final int FEEDER_SENSOR_FULL_PORT = 0; // 1;
    public static final int FEEDER_SENSOR_ENTRY_PORT = 0;

    // DRIVETRAIN Subsystem
    public static final double DRIVETRAIN_LENGTH_METERS = Units.inchesToMeters(19.75);
    public static final double DRIVETRAIN_WIDTH_METERS = Units.inchesToMeters(19.5);
    public static final int    DRIVETRAIN_PIGEON_ID = 29;
    //  Front Left Swerve Module
    public static final int    FRONT_LEFT_MODULE_DRIVE_MOTOR = 1;
    public static final int    FRONT_LEFT_MODULE_STEER_MOTOR = 11;
    public static final int    FRONT_LEFT_MODULE_STEER_ENCODER = 21;
    public static final double FRONT_LEFT_MODULE_STEER_OFFSET = -Math.toRadians(190.27); // Swervee Module Offset
    // Front Right Swerve Module
    public static final int FRONT_RIGHT_MODULE_DRIVE_MOTOR = 2;
    public static final int FRONT_RIGHT_MODULE_STEER_MOTOR = 12;
    public static final int FRONT_RIGHT_MODULE_STEER_ENCODER = 22;
    public static final double FRONT_RIGHT_MODULE_STEER_OFFSET = -Math.toRadians(261.88); // Swervee Module Offset
    // Back Left Swerve Module
    public static final int BACK_LEFT_MODULE_DRIVE_MOTOR = 3;
    public static final int BACK_LEFT_MODULE_STEER_MOTOR = 13;
    public static final int BACK_LEFT_MODULE_STEER_ENCODER = 23;
    public static final double BACK_LEFT_MODULE_STEER_OFFSET = -Math.toRadians(126.02); // Swervee Module Offset
    // Back Right Swerve Module
    public static final int BACK_RIGHT_MODULE_DRIVE_MOTOR = 4;
    public static final int BACK_RIGHT_MODULE_STEER_MOTOR = 14;
    public static final int BACK_RIGHT_MODULE_STEER_ENCODER = 24;
    public static final double BACK_RIGHT_MODULE_STEER_OFFSET = -Math.toRadians(263.67); // Swervee Module Offset

    // SHOOTER Subsystem
    public static final int HOOD_MOTOR_PORT = 0; // 14;
    public static final int FLYWHEEL_PRIMARY_MOTOR_PORT = 0; // 15;
    public static final int FLYWHEEL_SECONDARY_MOTOR_PORT = 0; // 9;
    public static final double HOOD_MANUAL_ADJUST_INTERVAL = Math.toRadians(0.5);
    public static final double FLYWHEEL_MANUAL_ADJUST_INTERVAL = Units.rotationsPerMinuteToRadiansPerSecond(25.0);
    public static final double HOOD_MOTOR_TO_HOOD_GEAR_RATIO = 1;
    public static final double HOOD_SHOOTING_ALLOWABLE_ERROR = Math.toRadians(0.5);
    public static final double HOOD_CLIMBING_ALLOWABLE_ERROR = Math.toRadians(1.0);
    public static final String SHOOTER_OFFSET_ENTRY_NAME = "Shooting Offset";
    public static final String HOOD_OFFSET_ENTRY_NAME = "Hood Offset";
    public static final String DRIVER_READOUT_TAB_NAME = "Driver Readout";
    public static final int CLAW_MOTOR_PORT = 17;

    // CHASSIS Subsystem
    public static final int CANDLE_ID = 27;  // the CAN ID for the FASTFD CAN Bus

    // Field measurements
    public static final double FIELD_LENGTH = Units.feetToMeters(54);
    public static final double FIELD_WIDTH = Units.feetToMeters(27);

    // Vision Stuff
    public static final String CAMERA_NAME = "photonvision";
    public static final Transform3d ROBOT_TO_CAMERA = new Transform3d(new Translation3d(.3, 0, 0.2),
            new Rotation3d(0, 0, 0));
    public static final Pose3d TAG_1_POSE3D = new Pose3d(FIELD_LENGTH, FIELD_WIDTH / 2, 4.5, new Rotation3d(0, 0, 180));
}
