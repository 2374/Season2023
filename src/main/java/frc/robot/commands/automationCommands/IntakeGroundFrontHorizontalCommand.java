package frc.robot.commands.automationCommands;

// import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.helperCommands.AlignArmFrontGroundCommand;
import frc.robot.commands.helperCommands.ControlIntakeCommand;
import frc.robot.subsystems.arm.ArmSubsystem;
import frc.robot.subsystems.ManipulatorSubsystem;

public class IntakeGroundFrontHorizontalCommand extends SequentialCommandGroup {
    public IntakeGroundFrontHorizontalCommand(ArmSubsystem armSubsystem, ManipulatorSubsystem manipulatorSubsystem) {
        addCommands(new AlignArmFrontGroundCommand(armSubsystem),
                new ControlIntakeCommand(manipulatorSubsystem, true));
    }
}
