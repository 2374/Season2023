package frc.robot.subsystems.expArm;

public enum ArmNamedPosition {
    // - add actual numbers for the positions
    CUBE_LOW(1.10 ,0.10,false),
    CUBE_MID(0.9117 ,0.8366,false, 0.1),
    CUBE_HIGH(1.377 ,1.123,false, 0.1),
    CONE_LOW(1.084 ,0.2082,false),
    CONE_MID(0.98 ,1.00,false, 0.2),
    CONE_HIGH(1.519 ,1.45,false, 0.25),
    SHELF(1.591 ,1.17,false),
    FLOOR(1.10 ,0.10,false),
    FLOOR_TIPPED_CONE(0.6539 ,0.07,true, 0.1),
    STOW(0.2002, 1.127 ,false),
    ;

    public final double safeY;
    public final double posX;
    public final double posY;
    public final boolean isReflex;
    
    ArmNamedPosition(double pos_x, double pos_y, boolean isReflex){
        this.posX = pos_x;
        this.posY = pos_y;
        this.isReflex = isReflex;
        this.safeY = 0.0;
    }

    ArmNamedPosition(double pos_x, double pos_y, boolean isReflex, double safeYOffset){
        this.posX = pos_x;
        this.posY = pos_y;
        this.isReflex = isReflex;
        this.safeY = pos_y + safeYOffset;
    }

    public ArmEndEffectorState get(){
        return new ArmEndEffectorState(this.posX, this.posY, this.isReflex);
    }
}
