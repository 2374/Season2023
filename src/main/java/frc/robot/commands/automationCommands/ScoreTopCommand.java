package frc.robot.commands.automationCommands;

// import java.util.function.BooleanSupplier;

// import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.arm.ArmSubsystem;
import frc.robot.Constants.ArmSetpoints;
import frc.robot.commands.helperCommands.ArmToSetPointCommand;
import frc.robot.commands.helperCommands.ControlIntakeCommand;
import frc.robot.subsystems.ManipulatorSubsystem;

public class ScoreTopCommand extends SequentialCommandGroup {

    public ScoreTopCommand(ArmSubsystem armSubsystem, ManipulatorSubsystem manipulatorSubsystem) {
        addCommands(new ArmToSetPointCommand(armSubsystem, ArmSetpoints.HIGH_SCORE));
        addCommands(new ControlIntakeCommand(manipulatorSubsystem, false));
    }
}
