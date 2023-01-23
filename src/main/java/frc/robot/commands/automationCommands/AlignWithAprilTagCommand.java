package frc.robot.commands.automationCommands;

import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.common.control.SimplePathBuilder;
import frc.common.control.Trajectory;
import frc.common.math.Rotation2;
import frc.common.math.Vector2;
import frc.robot.subsystems.DrivetrainSubsystem;

public class AlignWithAprilTagCommand extends CommandBase {

    private final DrivetrainSubsystem m_drivetrainSubsystem;

    public AlignWithAprilTagCommand(DrivetrainSubsystem drivetrainSubsystem) {
        this.m_drivetrainSubsystem = drivetrainSubsystem;
        addRequirements(m_drivetrainSubsystem);
    }

    @Override
    public void initialize() {
        AprilTag target = m_drivetrainSubsystem.getPhotonCameraWrapper().getFieldLayout().getTags()
                .get(m_drivetrainSubsystem.getPhotonCameraWrapper().getBestTarget().getFiducialId() - 1); // only works
                                                                                                          // if all tags
                                                                                                          // exist on
                                                                                                          // field
        double targetAngle = Math.toDegrees(target.pose.getRotation().getAngle());
        m_drivetrainSubsystem.getFollower().follow(new Trajectory(
                new SimplePathBuilder(
                        new Vector2(m_drivetrainSubsystem.getPose().getX(), m_drivetrainSubsystem.getPose().getY()),
                        Rotation2.fromDegrees(m_drivetrainSubsystem.getPose().getRotation().getDegrees()))
                        .lineTo(new Vector2(target.pose.getX() + Math.cos(targetAngle),
                                target.pose.getY() + Math.sin(targetAngle)), // 1 meter away from face
                                Rotation2.fromDegrees(targetAngle + 180))
                        .build(),
                DrivetrainSubsystem.TRAJECTORY_CONSTRAINTS, 0));
    }

    @Override
    public void end(boolean interupted) {
        m_drivetrainSubsystem.getFollower().cancel();
    }

    @Override
    public boolean isFinished() {
        return m_drivetrainSubsystem.getFollower().getCurrentTrajectory().isEmpty();
    }
}
