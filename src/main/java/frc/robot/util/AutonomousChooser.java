package frc.robot.util;

import java.util.function.BooleanSupplier;

// import java.util.function.BooleanSupplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.*;
import frc.robot.RobotContainer;
import frc.robot.commands.*;
import frc.robot.commands.automationCommands.goToAutoPositionCommand;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.common.control.Path;
import frc.common.control.SimplePathBuilder;
import frc.common.control.Trajectory;
import frc.common.math.Rotation2;
import frc.common.math.Vector2;

public class AutonomousChooser {
    private final AutonomousTrajectories trajectories;

    private final SendableChooser<AutonomousMode> autonomousModeChooser = new SendableChooser<>();

    public AutonomousChooser(AutonomousTrajectories trajectories) {
        this.trajectories = trajectories;

        autonomousModeChooser.setDefaultOption("Openhouse Autonomus", AutonomousMode.OPENHOUSE_AUTO);
        // autonomousModeChooser.addOption("1 meter B", AutonomousMode.ONE_METER_B);
        // autonomousModeChooser.addOption("Figure Eight", AutonomousMode.FIGURE_EIGHT);
        // autonomousModeChooser.addOption("Generic Back", AutonomousMode.GENERIC_BACK);
        autonomousModeChooser.addOption("Generic Score Back", AutonomousMode.GENERIC_SCORE_BACK);
        autonomousModeChooser.addOption("Score Engage", AutonomousMode.SCORE_ENGAGE);
        // autonomousModeChooser.addOption("Red Outer No Charge",
        // AutonomousMode.RED_OUTER_NO_CHARGE);
        // autonomousModeChooser.addOption("Red Outer Charge",
        // AutonomousMode.RED_OUTER_CHARGE);
        // autonomousModeChooser.addOption("Red Middle No Charge",
        // AutonomousMode.RED_MIDDLE_NO_CHARGE);
        // autonomousModeChooser.addOption("Red Middle Charge",
        // AutonomousMode.RED_MIDDLE_CHARGE);
        // autonomousModeChooser.addOption("Red Inner No Charge",
        // AutonomousMode.RED_INNER_NO_CHARGE);
        // autonomousModeChooser.addOption("Red Inner Charge",
        // AutonomousMode.RED_INNER_CHARGE);
        // autonomousModeChooser.addOption("Blue Outer No Charge",
        // AutonomousMode.BLUE_OUTER_NO_CHARGE);
        // autonomousModeChooser.addOption("Blue Outer Charge",
        // AutonomousMode.BLUE_OUTER_CHARGE);
        // autonomousModeChooser.addOption("Blue Middle No Charge",
        // AutonomousMode.BLUE_MIDDLE_NO_CHARGE);
        // autonomousModeChooser.addOption("Blue Middle Charge",
        // AutonomousMode.BLUE_MIDDLE_CHARGE);
        // autonomousModeChooser.addOption("Blue Inner No Charge",
        // AutonomousMode.BLUE_INNER_NO_CHARGE);
        // autonomousModeChooser.addOption("Blue Inner Charge",
        // AutonomousMode.BLUE_INNER_CHARGE);
    }

    public SendableChooser<AutonomousMode> getModeChooser() {
        return autonomousModeChooser;
    }

    public Command getOpenhouseAuto(RobotContainer container) {
        SequentialCommandGroup command = new SequentialCommandGroup();

        command.addCommands(
                // TODO Align with Apriltags (at position 1 pickup)
                resetRobotPose(container, new Pose2d(2, .5, new Rotation2d(Math.PI))),
                visionReset(container),
                goToAutoPosition(container, 1),
                gotoSetpoint(container, () -> container.getArmSubsystem().setpointUP()),
                backWhileOuttake(container)
        // TODO forwardWhileIntake
        // gotoSetpoint(container, () -> container.getArmSubsystem().setpointBACK())
        // TODO backward
        // TODO Rotate 180 degrees
        // TODO Align with Apriltags (at position 2 deposit)
        // gotoSetpoint(container, () -> container.getArmSubsystem().setpointUP()),
        // backWhileOuttake(container),
        // gotoSetpoint(container, () -> container.getArmSubsystem().setpointBACK()),
        // TODO Rotate 180 degrees
        // TODO Align with Apriltags (at position 1 pickup)
        // gotoSetpoint(container, () -> container.getArmSubsystem().setpointRIGHT()),
        // TODO forwardWhileIntake
        // gotoSetpoint(container, () -> container.getArmSubsystem().setpointBACK())
        // TODO backward
        // TODO Rotate 180 degrees
        // TODO Align with Apriltags (at position 2 deposit)
        // gotoSetpoint(container, () -> container.getArmSubsystem().setpointRIGHT()),
        // backWhileOuttake(container),
        // gotoSetpoint(container, () -> container.getArmSubsystem().setpointBACK()),

        // TODO Align with Apriltags (at position 2 pickup)
        // gotoSetpoint(container, () -> container.getArmSubsystem().setpointUP()),
        // TODO forwardWhileIntake
        // gotoSetpoint(container, () -> container.getArmSubsystem().setpointBACK())
        // TODO backward
        // TODO Rotate 180 degrees
        // TODO Align with Apriltags (at position 1 deposit)
        // gotoSetpoint(container, () -> container.getArmSubsystem().setpointUP()),
        // backWhileOuttake(container),
        // gotoSetpoint(container, () -> container.getArmSubsystem().setpointBACK()),
        // TODO Rotate 180 degrees
        // TODO Align with Apriltags (at position 2 pickup)
        // gotoSetpoint(container, () -> container.getArmSubsystem().setpointRIGHT()),
        // TODO forwardWhileIntake
        // gotoSetpoint(container, () -> container.getArmSubsystem().setpointBACK())
        // TODO backward
        // TODO Rotate 180 degrees
        // TODO Align with Apriltags (at position 1 deposit)
        // gotoSetpoint(container, () -> container.getArmSubsystem().setpointRIGHT()),
        // backWhileOuttake(container),
        // gotoSetpoint(container, () -> container.getArmSubsystem().setpointBACK()),
        );

        return command;
    }

    public Command getGenericScoreBackAuto(RobotContainer container) {
        SequentialCommandGroup command = new SequentialCommandGroup();

        command.addCommands(
                resetRobotPose(container),
                gotoSetpoint(container, () -> container.getArmSubsystem().setpointUP()),
                backWhileOuttake(container),
                gotoSetpoint(container, () -> container.getArmSubsystem().setpointBACK()),
                followLine(container, -3.5, 0),
                new RunCommand(() -> {
                    if ((container.getDrivetrain().getYaw() - 90) % 360 > 180) {
                        container.getDrivetrain().drive(
                                new ChassisSpeeds(0, 0,
                                        2 * ((container.getDrivetrain().getYaw() - 90 + 180) % 360 / 360) + 0.3));
                    } else {
                        container.getDrivetrain().drive(
                                new ChassisSpeeds(0, 0,
                                        -2 * ((container.getDrivetrain().getYaw() - 90 + 180) % 360 / 360) - 0.3));
                    }
                }).until(new BooleanSupplier() {
                    public boolean getAsBoolean() {
                        return Math.abs(((container.getDrivetrain().getYaw() - 90) % 360) - 180) < 5;
                    };
                }),
                resetRobotPose(container));
        return command;
    }

    public Command getScoreEngageAuto(RobotContainer container) {
        SequentialCommandGroup command = new SequentialCommandGroup();

        command.addCommands(
                resetRobotPose(container),
                gotoSetpoint(container, () -> container.getArmSubsystem().setpointUP()),
                backWhileOuttake(container),
                gotoSetpoint(container, () -> container.getArmSubsystem().setpointBACK()),
                mountAndBalence(container));
        return command;
    }

    private Command goToAutoPosition(RobotContainer container, int number) {
        return new goToAutoPositionCommand(container.getDrivetrain(), number);
    }

    private Command visionReset(RobotContainer container) {
        return new WaitCommand(1.5).andThen(new InstantCommand(() -> container.getDrivetrain().useVisionPosition()));
    }

    private Command backWhileOuttake(RobotContainer container) {
        return new ParallelCommandGroup(new InstantCommand(() -> container.getManipulatorSubsystem().outtake(),
                container.getManipulatorSubsystem()), followLine(container, -0.3, 0));
    }

    private Command gotoSetpoint(RobotContainer container,
            Runnable buttonDirectionMethod) {
        return new InstantCommand(buttonDirectionMethod).andThen(new WaitCommand(0.1))
                .andThen(new WaitUntilCommand(() -> container.getArmSubsystem().bothJointsAtSetpoint()));
    }

    private Command follow(RobotContainer container, Trajectory trajectory) {
        return new FollowTrajectoryCommand(container.getDrivetrain(), trajectory);
    }

    private Command followLine(RobotContainer container, double x, double y,
            double rotationDegrees) {
        return new FollowTrajectoryCommand(container.getDrivetrain(),
                new Trajectory(
                        new SimplePathBuilder(new Vector2(0, 0), Rotation2.ZERO)
                                .lineTo(new Vector2(x, y), Rotation2.fromDegrees(rotationDegrees)).build(),
                        DrivetrainSubsystem.TRAJECTORY_CONSTRAINTS, 0.1));
    }

    private Command followLine(RobotContainer container, double x, double y) {
        return followLine(container, x, y, 0);
    }

    private Command mountAndBalence(RobotContainer container) {
        return new InstantCommand(() -> container.getDrivetrain().drive(new ChassisSpeeds(-1.6, 0, 0)),
                container.getDrivetrain()).andThen(
                        new WaitUntilCommand(new BooleanSupplier() {
                            public boolean getAsBoolean() {
                                return container.getDrivetrain().getPitch() < -10;
                            };
                        }))
                .andThen(
                        new WaitCommand(.2))
                .andThen(
                        new InstantCommand(() -> container.getDrivetrain().drive(new ChassisSpeeds(-0.6, 0, 0)),
                                container.getDrivetrain()))
                .andThen(
                        new WaitUntilCommand(new BooleanSupplier() {
                            public boolean getAsBoolean() {
                                return container.getDrivetrain().getPitch() > -8;
                            };
                        }))
                .andThen(
                        new RunCommand(() -> container.getDrivetrain().autoBalenceTick(),
                                container.getDrivetrain()));
    }

    // private Command gotoSetPoint(RobotContainer container, Setpoint setpoint) {
    // return new ArmToSetPointCommand(container.getArmSubsystem(), setpoint);
    // }

    public Command resetRobotPose(RobotContainer container, Trajectory trajectory) {
        Path.State start = trajectory.getPath().calculate(0.0);
        return new InstantCommand(() -> container.getDrivetrain().setPose(new Pose2d(start.getPosition().x,
                start.getPosition().y, new Rotation2d(start.getRotation().toRadians()))));
    }

    public Command resetRobotPose(RobotContainer container, Pose2d pose) {
        return new InstantCommand(() -> container.getDrivetrain().setPose(pose));
    }

    public Command resetRobotPose(RobotContainer container) {
        return new InstantCommand(
                () -> container.getDrivetrain().setPose(new Pose2d(0, 0, Rotation2d.fromDegrees(0))));
    }

    // Handler to determine what command was requested for the autonmous routine to
    // execute
    public Command getCommand(RobotContainer container) {
        switch (autonomousModeChooser.getSelected()) {
            case OPENHOUSE_AUTO:
                return getOpenhouseAuto(container);
            case GENERIC_SCORE_BACK:
                return getGenericScoreBackAuto(container);
            case SCORE_ENGAGE:
                return getScoreEngageAuto(container);
            default:
                break;
        }
        return new InstantCommand();
    }

    private enum AutonomousMode {
        OPENHOUSE_AUTO, GENERIC_SCORE_BACK, SCORE_ENGAGE
    }
}