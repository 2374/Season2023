package frc.robot.commands.helperCommands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ArmSubsystem;

public class AlignArmFrontGroundCommand extends CommandBase {

    private final ArmSubsystem m_ArmSubsystem;

    public AlignArmFrontGroundCommand(ArmSubsystem armSubsystem) {
        m_ArmSubsystem = armSubsystem;
        addRequirements(m_ArmSubsystem);
    }

    @Override
    public void initialize() {
        System.out.println("Aligning arm");
    }
}
