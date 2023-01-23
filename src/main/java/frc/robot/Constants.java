package frc.robot;

import java.util.ArrayList;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;

public class Constants {
    public static final String CANIVORE_CAN_BUS_NAME = "FastFD";
    public static final String RIO_CAN_BUS_NAME = "rio";
    public static final String DRIVETRAIN_CAN_BUS_NAME = CANIVORE_CAN_BUS_NAME;

    public static final int CONTROLLER_PORT = 0;

    public static final int ARM_MOTOR_PORT = 8;

    public static final int FEEDER_SENSOR_FULL_PORT = 0; // 1;
    public static final int FEEDER_SENSOR_ENTRY_PORT = 0;

    public static final double DRIVETRAIN_TRACKWIDTH_METERS = Units.inchesToMeters(19.5);
    public static final double DRIVETRAIN_WHEELBASE_METERS = Units.inchesToMeters(19.75);
    public static final int DRIVETRAIN_PIGEON_ID = 29; // 1;

    public static final int FRONT_LEFT_MODULE_DRIVE_MOTOR = 1; // 7;
    public static final int FRONT_LEFT_MODULE_STEER_MOTOR = 11; // 8;
    public static final int FRONT_LEFT_MODULE_STEER_ENCODER = 21; // 4;
    // public static final double FRONT_LEFT_MODULE_STEER_OFFSET =
    // -Math.toRadians(127.64 + 180.0); // color tag
    // public static final double FRONT_LEFT_MODULE_STEER_OFFSET =
    // -Math.toRadians(90.87 + 180.0);
    public static final double FRONT_LEFT_MODULE_STEER_OFFSET = -Math.toRadians(10.27); // label tag

    public static final int FRONT_RIGHT_MODULE_DRIVE_MOTOR = 2; // 3;
    public static final int FRONT_RIGHT_MODULE_STEER_MOTOR = 12; // 4;
    public static final int FRONT_RIGHT_MODULE_STEER_ENCODER = 22; // 3;
    // public static final double FRONT_RIGHT_MODULE_STEER_OFFSET =
    // -Math.toRadians(65.96 + 180.0); // color tag
    // public static final double FRONT_RIGHT_MODULE_STEER_OFFSET =
    // -Math.toRadians(35.59 + 180.0);
    public static final double FRONT_RIGHT_MODULE_STEER_OFFSET = -Math.toRadians(81.88); // label tag

    public static final int BACK_LEFT_MODULE_DRIVE_MOTOR = 3; // 5;
    public static final int BACK_LEFT_MODULE_STEER_MOTOR = 13; // 6;
    public static final int BACK_LEFT_MODULE_STEER_ENCODER = 23; // 2;
    // public static final double BACK_LEFT_MODULE_STEER_OFFSET =
    // -Math.toRadians(93.24 + 180.0); // color tag
    // public static final double BACK_LEFT_MODULE_STEER_OFFSET =
    // -Math.toRadians(110.21 + 180.0);
    public static final double BACK_LEFT_MODULE_STEER_OFFSET = -Math.toRadians(306.02); // label tag

    public static final int BACK_RIGHT_MODULE_DRIVE_MOTOR = 4; // 1;
    public static final int BACK_RIGHT_MODULE_STEER_MOTOR = 14; // 2;
    public static final int BACK_RIGHT_MODULE_STEER_ENCODER = 24; // 1;
    // public static final double BACK_RIGHT_MODULE_STEER_OFFSET =
    // -Math.toRadians(146.59 + 180.0); // color tag
    // public static final double BACK_RIGHT_MODULE_STEER_OFFSET =
    // -Math.toRadians(78.13 + 180.0);
    public static final double BACK_RIGHT_MODULE_STEER_OFFSET = -Math.toRadians(263.67); // label tag

    public static final int HOOD_MOTOR_PORT = 0; // 14;
    public static final int FLYWHEEL_PRIMARY_MOTOR_PORT = 0; // 15;
    public static final int FLYWHEEL_SECONDARY_MOTOR_PORT = 0; // 9;

    public static final double HOOD_MANUAL_ADJUST_INTERVAL = Math.toRadians(0.5);
    public static final double FLYWHEEL_MANUAL_ADJUST_INTERVAL = Units.rotationsPerMinuteToRadiansPerSecond(25.0);

    public static final double HOOD_MOTOR_TO_HOOD_GEAR_RATIO = 1;
    public static final double HOOD_SHOOTING_ALLOWABLE_ERROR = Math.toRadians(0.5);
    public static final double HOOD_CLIMBING_ALLOWABLE_ERROR = Math.toRadians(1.0);

    public static final double ARM_LIMIT = 0;

    public static final String SHOOTER_OFFSET_ENTRY_NAME = "Shooting Offset";
    public static final String HOOD_OFFSET_ENTRY_NAME = "Hood Offset";
    public static final String DRIVER_READOUT_TAB_NAME = "Driver Readout";
    public static final int CLAW_MOTOR_PORT = 17;
    public static final int CANDLE_ID = 27;

    // Field measurements
    public static final double FIELD_LENGTH = Units.feetToMeters(54);
    public static final double FIELD_WIDTH = Units.feetToMeters(27);

    // Vision Stuff
    public static final String CAMERA_NAME = "photonCamera";
    public static final Transform3d ROBOT_TO_CAMERA = new Transform3d(new Translation3d(0, 0, 0.5),
            new Rotation3d(0, 0, 0));
    public static final Pose3d TAG_1_POSE3D = new Pose3d(FIELD_LENGTH, FIELD_WIDTH / 2, 0, new Rotation3d(0, 0, 180));
}
