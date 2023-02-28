package frc.robot.commands.helperCommands;

import java.util.function.BooleanSupplier;

// import com.swervedrivespecialties.swervelib.rev.NeoSteerControllerFactoryBuilder.ControllerImplementation;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ManipulatorSubsystem;

public class ControlIntakeCommand extends CommandBase {
    private final ManipulatorSubsystem m_ManipulatorSubsystem;
    private final boolean in;

    /**
     * Starts the Manipulator
     * 
     * @param manipulatorSubsystem The manipulator subsystem
     * @param in                   Intake or Outtake
     */
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
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
