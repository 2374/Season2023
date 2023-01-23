package frc.robot.commands.automationCommands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DrivetrainSubsystem;

public class AutoBalenceCommand extends CommandBase {

    private final DrivetrainSubsystem m_drivetrainSubsystem;

    public AutoBalenceCommand(DrivetrainSubsystem drivetrainSubsystem) {
        this.m_drivetrainSubsystem = drivetrainSubsystem;
        addRequirements(m_drivetrainSubsystem);
    }

    @Override
    public void initialize() {
    }

    @Override
    public void execute() {
        m_drivetrainSubsystem.autoBalenceTick();
    }

    @Override
    public void end(boolean interupted) {
    }
}
