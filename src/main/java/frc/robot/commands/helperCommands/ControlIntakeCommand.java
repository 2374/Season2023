package frc.robot.commands.helperCommands;

import java.util.function.BooleanSupplier;

// import com.swervedrivespecialties.swervelib.rev.NeoSteerControllerFactoryBuilder.ControllerImplementation;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ManipulatorSubsystem;

public class ControlIntakeCommand extends CommandBase {
    private final ManipulatorSubsystem m_ManipulatorSubsystem;
    private final boolean in;

    public ControlIntakeCommand(ManipulatorSubsystem manipulatorSubsystem, boolean in) {
        m_ManipulatorSubsystem = manipulatorSubsystem;
        this.in = in;
        addRequirements(m_ManipulatorSubsystem);
    }

    @Override
    public void initialize() {
        if (in) {
            m_ManipulatorSubsystem.intake();
        } else {
            m_ManipulatorSubsystem.outtake();
        }
        m_ManipulatorSubsystem.activate();
    }

    @Override
    public void end(boolean interrupted) {
        System.out.println("Stop");
        m_ManipulatorSubsystem.stoptake();
        m_ManipulatorSubsystem.deactivate();
    }
}
