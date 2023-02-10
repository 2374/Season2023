package frc.robot.commands.helperCommands;

// import com.swervedrivespecialties.swervelib.rev.NeoSteerControllerFactoryBuilder.ControllerImplementation;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ManipulatorSubsystem;

public class ControlIntakeCommand extends CommandBase {
    private final ManipulatorSubsystem m_ManipulatorSubsystem;

    public ControlIntakeCommand(ManipulatorSubsystem manipulatorSubsystem) {
        m_ManipulatorSubsystem = manipulatorSubsystem;
        addRequirements(m_ManipulatorSubsystem);
    }
}
