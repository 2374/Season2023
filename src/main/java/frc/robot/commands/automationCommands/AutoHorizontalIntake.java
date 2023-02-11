package frc.robot.commands.automationCommands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.common.control.SimplePathBuilder;
import frc.common.control.Trajectory;
import frc.common.math.Rotation2;
import frc.common.math.Vector2;
import frc.robot.commands.FollowTrajectoryCommand;
import frc.robot.commands.helperCommands.ControlIntakeCommand;
import frc.robot.commands.helperCommands.armToRestCommand;
import frc.robot.subsystems.arm.ArmSubsystem;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.ManipulatorSubsystem;

public class AutoHorizontalIntake extends SequentialCommandGroup {
    public AutoHorizontalIntake(DrivetrainSubsystem drivetrainSubsystem, ArmSubsystem armSubsystem,
            ManipulatorSubsystem manipulatorSubsystem, boolean cone) {
        addCommands(
                new FollowTrajectoryCommand(drivetrainSubsystem,
                        new Trajectory(new SimplePathBuilder(new Vector2(0, 0), Rotation2.ZERO)
                                .lineTo(new Vector2(.25, 0)).build(), DrivetrainSubsystem.TRAJECTORY_CONSTRAINTS, 0))
                        .alongWith(new ControlIntakeCommand(manipulatorSubsystem, true, cone)),
                new armToRestCommand(armSubsystem));
    }
}
