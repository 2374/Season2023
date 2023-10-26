package frc.robot.commands.automationCommands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.common.control.SimplePathBuilder;
import frc.common.control.Trajectory;
import frc.common.math.Rotation2;
import frc.common.math.Vector2;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.util.AutonomousTrajectories;

public class goToAutoPositionCommand extends CommandBase {
    private DrivetrainSubsystem m_DrivetrainSubsystem;
    private int num;

    public goToAutoPositionCommand(DrivetrainSubsystem subsystem, int number) {
        m_DrivetrainSubsystem = subsystem;
        num = number;
        addRequirements(m_DrivetrainSubsystem);
    }

    @Override
    public void initialize() {
        Pose2d finalPose;
        switch (num) {
            case 1:
                finalPose = new Pose2d(1.25, .5, new Rotation2d(Math.PI));
                break;
            case 2:
                finalPose = new Pose2d(2, .5, new Rotation2d(Math.PI));
                break;
            case 3:
                finalPose = new Pose2d(2.5, .5, new Rotation2d(0));
                break;
            default:
                finalPose = new Pose2d(2, .5, new Rotation2d(0));
                break;
        }

        System.out.println(m_DrivetrainSubsystem.getPose());
        System.out.println(finalPose);

        m_DrivetrainSubsystem.getFollower().follow(new Trajectory(
                new SimplePathBuilder(Vector2.translation2dToVector2(m_DrivetrainSubsystem.getPose().getTranslation()),
                        Rotation2.fromDegrees(m_DrivetrainSubsystem.getPose().getRotation().getDegrees())).lineTo(
                                new Vector2(finalPose.getX(), finalPose.getY()),
                                Rotation2.fromDegrees(finalPose.getRotation().getDegrees()))
                        .build(),
                DrivetrainSubsystem.TRAJECTORY_CONSTRAINTS,
                AutonomousTrajectories.SAMPLE_DISTANCE));
    }

    @Override
    public boolean isFinished() {
        return m_DrivetrainSubsystem.getFollower().getCurrentTrajectory().isEmpty();
    }
}
