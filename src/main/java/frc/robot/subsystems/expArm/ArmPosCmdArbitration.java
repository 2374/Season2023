package frc.robot.subsystems.expArm;

public class ArmPosCmdArbitration {

    public static ArmEndEffectorState arbitrate(ArmEndEffectorState manCmd, ArmEndEffectorState pathCmd){
        // - any additinoal inputs needed?
        // - pick and return the right position
        return pathCmd;
    }
    
}
