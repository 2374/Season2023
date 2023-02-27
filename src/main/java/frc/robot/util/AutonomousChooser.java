package frc.robot.util;

// import java.util.function.BooleanSupplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.*;
import frc.robot.RobotContainer;
import frc.robot.commands.*;
import frc.robot.commands.automationCommands.AutoHorizontalIntake;
import frc.robot.commands.automationCommands.ScoreTopCommand;
import frc.robot.commands.helperCommands.AlignArmFrontGroundCommand;
import frc.robot.commands.helperCommands.ArmToSetPointCommand;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.arm.Setpoint;
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

        autonomousModeChooser.setDefaultOption("1 meter F", AutonomousMode.ONE_METER_F);
        autonomousModeChooser.addOption("1 meter B", AutonomousMode.ONE_METER_B);
        // autonomousModeChooser.addOption("Figure Eight", AutonomousMode.FIGURE_EIGHT);
        autonomousModeChooser.addOption("Red Outer No Charge", AutonomousMode.RED_OUTER_NO_CHARGE);
        autonomousModeChooser.addOption("Red Outer Charge", AutonomousMode.RED_OUTER_CHARGE);
        autonomousModeChooser.addOption("Red Middle No Charge", AutonomousMode.RED_MIDDLE_NO_CHARGE);
        autonomousModeChooser.addOption("Red Middle Charge", AutonomousMode.RED_MIDDLE_CHARGE);
        autonomousModeChooser.addOption("Red Inner No Charge", AutonomousMode.RED_INNER_NO_CHARGE);
        autonomousModeChooser.addOption("Red Inner  Charge", AutonomousMode.RED_INNER_CHARGE);
        autonomousModeChooser.addOption("Blue Outer No Charge", AutonomousMode.BLUE_OUTER_NO_CHARGE);
        autonomousModeChooser.addOption("Blue Outer Charge", AutonomousMode.BLUE_OUTER_CHARGE);
        autonomousModeChooser.addOption("Blue Middle No Charge", AutonomousMode.BLUE_MIDDLE_NO_CHARGE);
        autonomousModeChooser.addOption("Blue Middle Charge", AutonomousMode.BLUE_MIDDLE_CHARGE);
        autonomousModeChooser.addOption("Blue Inner No Charge", AutonomousMode.BLUE_INNER_NO_CHARGE);
        autonomousModeChooser.addOption("Blue Inner Charge", AutonomousMode.BLUE_INNER_CHARGE);
    }

    public SendableChooser<AutonomousMode> getModeChooser() {
        return autonomousModeChooser;
    }

    public Command getOneMeterFAuto(RobotContainer container) {
        SequentialCommandGroup command = new SequentialCommandGroup();

        resetRobotPose(command, container, trajectories.getOneMeterF());

        command.addCommands(follow(container, trajectories.getOneMeterF()));

        return command;
    }

    public Command getOneMeterBAuto(RobotContainer container) {
        SequentialCommandGroup command = new SequentialCommandGroup();

        resetRobotPose(command, container, trajectories.getOneMeterB());

        command.addCommands(follow(container, trajectories.getOneMeterB()));

        return command;
    }

    public Command getFigureEightAuto(RobotContainer container) {
        SequentialCommandGroup command = new SequentialCommandGroup();

        resetRobotPose(command, container, trajectories.getFigureEight());

        command.addCommands(follow(container, trajectories.getFigureEight()));

        return command;
    }

    public Command getRedOuterNoChargeCommand(RobotContainer container) {
        SequentialCommandGroup command = new SequentialCommandGroup();

        // assumes robot starts in front of the outer scoring poles
        resetRobotPose(command, container, trajectories.getFourPointNineFiveMeterF());
        // score the cone on the top row
        // topConeScore(command, container);
        // move the arm to rest while moving to pickup the cube 4.7 meters toward the
        // middle of the field
        command.addCommands(follow(container, trajectories.getFourPointNineFiveMeterF()));
        // fourPointSevenMeterWithFrontArmMovement(command, container);
        // intake the cone from the ground
        // THIS IS BROKEN intakeGroundThenRest(command, container);
        // move back toward the scoring area and slide toward the middle to align to
        // score
        command.addCommands(follow(container,
                trajectories.getFourPointSevenMeterB()),
                follow(container, trajectories.getSideOneMeter()));
        // score the top row of the 2nd cone in from the outer wall
        // topConeScore(command, container);

        return command;
    }

    public Command getRedOuterChargeCommand(RobotContainer container) {
        SequentialCommandGroup command = new SequentialCommandGroup();

        // assumes robot starts in front of the outer scoring poles
        resetRobotPose(command, container, trajectories.getFourPointSevenMeterB());
        // score the cone on the top row
        // topConeScore(command, container);
        // move 4.7 meters toward the center of the field so as to clear the charge
        // station
        // fourPointSevenMeterWithFrontArmMovement(command, container);
        // move 2 meters toward the middle of the charge station to align for climbing
        // move 1.5 meeters to get onto charge station
        // auto balance

        return command;
    }

    // private void scoreTop(SequentialCommandGroup command, RobotContainer
    // container) {
    // command.addCommands(new ScoreTopCommand(container.getArmSubsystem(),
    // container.getManipulatorSubsystem()));
    // }

    // private void fourPointSevenMeterWithFrontArmMovement(SequentialCommandGroup
    // command, RobotContainer container) {
    // command.addCommands(follow(container, trajectories.getFourPointSevenMeterB())
    // .alongWith(new AlignArmFrontGroundCommand(container.getArmSubsystem())));
    // }

    // private void intakeGroundThenRest(SequentialCommandGroup command,
    // RobotContainer container) {
    // command.addCommands(new AutoHorizontalIntake(container.getDrivetrain(),
    // container.getArmSubsystem(),
    // container.getManipulatorSubsystem(), true));
    // }

    private Command follow(RobotContainer container, Trajectory trajectory) {
        return new FollowTrajectoryCommand(container.getDrivetrain(), trajectory);
    }

    private Command followLine(RobotContainer container, int x, int y, int rotationDegrees) {
        return new FollowTrajectoryCommand(container.getDrivetrain(),
                new Trajectory(
                        new SimplePathBuilder(new Vector2(0, 0), Rotation2.ZERO)
                                .lineTo(new Vector2(x, y), Rotation2.fromDegrees(rotationDegrees)).build(),
                        DrivetrainSubsystem.TRAJECTORY_CONSTRAINTS, 0));
    }

    private Command followLine(RobotContainer container, int x, int y) {
        return followLine(container, x, y, 0);
    }

    // private Command gotoSetPoint(RobotContainer container, Setpoint setpoint) {
    // return new ArmToSetPointCommand(container.getArmSubsystem(), setpoint);
    // }

    public void resetRobotPose(SequentialCommandGroup command, RobotContainer container, Trajectory trajectory) {
        Path.State start = trajectory.getPath().calculate(0.0);
        command.addCommands(new InstantCommand(() -> container.getDrivetrain().setPose(new Pose2d(start.getPosition().x,
                start.getPosition().y, new Rotation2d(start.getRotation().toRadians())))));
    }

    // Handler to determine what command was requested for the autonmous routine to
    // execute
    public Command getCommand(RobotContainer container) {
        switch (autonomousModeChooser.getSelected()) {
            case ONE_METER_F:
                return getOneMeterFAuto(container);
            case ONE_METER_B:
                return getOneMeterBAuto(container);
            case FIGURE_EIGHT:
                return getFigureEightAuto(container);
            case RED_OUTER_NO_CHARGE:
                return getRedOuterNoChargeCommand(container);
            case RED_OUTER_CHARGE:
                return getRedOuterChargeCommand(container);
            case RED_MIDDLE_NO_CHARGE:
                return getRedOuterNoChargeCommand(container);
            case RED_MIDDLE_CHARGE:
                return getRedOuterChargeCommand(container);
            case RED_INNER_NO_CHARGE:
                return getRedOuterNoChargeCommand(container);
            case RED_INNER_CHARGE:
                return getRedOuterChargeCommand(container);
            case BLUE_OUTER_NO_CHARGE:
                return getRedOuterChargeCommand(container);
            case BLUE_OUTER_CHARGE:
                return getRedOuterChargeCommand(container);
            case BLUE_MIDDLE_NO_CHARGE:
                return getRedOuterNoChargeCommand(container);
            case BLUE_MIDDLE_CHARGE:
                return getRedOuterChargeCommand(container);
            case BLUE_INNER_NO_CHARGE:
                return getRedOuterNoChargeCommand(container);
            case BLUE_INNER_CHARGE:
                return getRedOuterChargeCommand(container);
            default:
                break;
        }
        return new InstantCommand();
    }

    private enum AutonomousMode {
        ONE_METER_F, ONE_METER_B, FIGURE_EIGHT, RED_OUTER_NO_CHARGE, RED_OUTER_CHARGE, RED_MIDDLE_NO_CHARGE,
        RED_MIDDLE_CHARGE, RED_INNER_NO_CHARGE, RED_INNER_CHARGE,
        BLUE_OUTER_NO_CHARGE, BLUE_OUTER_CHARGE, BLUE_MIDDLE_NO_CHARGE, BLUE_MIDDLE_CHARGE, BLUE_INNER_NO_CHARGE,
        BLUE_INNER_CHARGE
    }
}