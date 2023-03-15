package frc.robot.subsystems.expArm;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotController;
import frc.robot.Constants;
import frc.hardwareWrappers.AbsoluteEncoder.WrapperedAbsoluteEncoder;
import frc.hardwareWrappers.AbsoluteEncoder.WrapperedAbsoluteEncoder.AbsoluteEncType;
import frc.lib.Util.FunctionGenerator;
import frc.lib.Util.MapLookup2D;
import frc.robot.ArmTelemetry;

public class ArmControl {

	private static ArmControl inst = null;
	public static synchronized ArmControl getInstance() {
		if(inst == null)
			inst = new ArmControl();
		return inst;
	}

    boolean releaseBrakeCmd;

    MotorControlShoulder mb;
    MotorControlElbow ms;

    ArmAngularState curMeasAngularStates;
    ArmEndEffectorState curDesState;
    ArmEndEffectorState prevDesState;

    ArmPathPlanner pp;
    ArmManPosition mp;
    ArmConePlaceOffset cpo;

    ArmSoftLimits asl;

    WrapperedAbsoluteEncoder shoulderEncoder = new WrapperedAbsoluteEncoder(AbsoluteEncType.RevThroughBore, "Shoulder", Constants.ARM_SHOULDER_ENC_IDX, Constants.ARM_SHOULDER_ENCODER_MOUNT_OFFSET_RAD, false);
    WrapperedAbsoluteEncoder elbowEncoder = new WrapperedAbsoluteEncoder(AbsoluteEncType.RevThroughBore, "Elbow", Constants.ARM_ELBOW_ENC_IDX, Constants.ARM_ELBOW_ENCODER_MOUNT_OFFSET_RAD, true);

    // Test mode tools
    // These help us inject specific waveforms into swerve modules to calibrate and test them.
    FunctionGenerator shoulderFG;
    FunctionGenerator elbowFG;

    MapLookup2D speedLimitMap;

    private ArmControl(){
        mb = new MotorControlShoulder();
        ms = new MotorControlElbow();
        pp = new ArmPathPlanner();
        mp = new ArmManPosition();
        cpo = new ArmConePlaceOffset();
        asl = new ArmSoftLimits();
        curMeasAngularStates = new ArmAngularState(0,0);
        curDesState = ArmNamedPosition.STOW.get();
        prevDesState = ArmNamedPosition.STOW.get();

        shoulderFG = new FunctionGenerator("arm_shoulder", "deg");
        elbowFG = new FunctionGenerator("arm_elbow", "deg");

        speedLimitMap = new MapLookup2D();
        speedLimitMap.insertNewPoint(Constants.WHEEL_BASE_HALF_LENGTH_M + 0.4, 0.5);
        speedLimitMap.insertNewPoint(Constants.WHEEL_BASE_HALF_LENGTH_M + 0.3, 0.75);
        speedLimitMap.insertNewPoint(Constants.WHEEL_BASE_HALF_LENGTH_M + 0.2, 0.75);
        speedLimitMap.insertNewPoint(Constants.WHEEL_BASE_HALF_LENGTH_M + 0.1, 1.0);
    }

    public void setInactive(){
        this.setOpCmds(0.0, 0.0, ArmNamedPosition.STOW, false, 0.0);
    }

    public void setOpCmds(double desXVel, double desYVel, ArmNamedPosition posCmd, boolean posCmdActive, double vertOffsetCmd){
        var manVelCmd = !posCmdActive;
        mp.setOpVelCmds(manVelCmd, desXVel, desYVel);
        pp.setCommand(posCmdActive, posCmd);
        cpo.setCmd(vertOffsetCmd);
    }

    private ArmEndEffectorState updateMeasState(){
        // Meas state and end effector position
        shoulderEncoder.update();
        elbowEncoder.update();
        shoulderEncoder.isFaulted();
        elbowEncoder.isFaulted();
        var shoulderAngleDeg = Units.radiansToDegrees(shoulderEncoder.getAngle_rad());
        var elbowAngleDeg = Units.radiansToDegrees(elbowEncoder.getAngle_rad());
        curMeasAngularStates = new ArmAngularState(shoulderAngleDeg, elbowAngleDeg);      
        return ArmKinematics.forward(curMeasAngularStates);   
    }

    public void update(){

        ArmEndEffectorState curMeasState = updateMeasState();

        curDesState = new ArmEndEffectorState();
        ArmEndEffectorState curDesStateWithOffset;

        if(DriverStation.isDisabled()){
            // While disabled, by default, we just maintain measured state
            curDesState.x = curMeasState.x;
            curDesState.y = curMeasState.y;
            curDesState.isReflex = curMeasState.isReflex;
        } else {
            // While endabled, by default, the next desired state
            // is just the position of the desired state position, with zero velocity
            curDesState.x = prevDesState.x;
            curDesState.y = prevDesState.y;
            curDesState.isReflex = prevDesState.isReflex;
        }

        // Allow the brakes to be released in disabled for easy manipulation in the pit
        if(DriverStation.isDisabled() && RobotController.getUserButton()){
            mb.setBrakeMode(false);
            ms.setBrakeMode(false);
        } else {
            mb.setBrakeMode(true);
            ms.setBrakeMode(true);
        }

        // Allow the path planner top (optionally) modify the desired state
        curDesState = pp.update(curDesState);

        // Allow the manual motion module to (optionally) modify the desired state
        curDesState = mp.update(curDesState);

        // Allow the offset module to (optionally) modify the desired state
        curDesStateWithOffset = cpo.update(curDesState);

        //Apply soft limits
        var curDesStateLimited = asl.applyLimit(curDesStateWithOffset);

        // Apply kinematics to get linkge positions
        var curDesAngularStates = ArmKinematics.inverse(curDesStateLimited);

        // Send desired state to the motor control
        mb.setCmd(curDesAngularStates);
        if(isFaulted() == true){
            mb.update(curMeasAngularStates, false);
        } else {
            mb.update(curMeasAngularStates, true);
        }

        ms.setCmd(curDesAngularStates);
        if(isFaulted() == true){
            ms.update(curMeasAngularStates, false);
        } else {
            ms.update(curMeasAngularStates, true);
        }


        // Update telemetry
        ArmTelemetry.getInstance().setDesired(curDesStateLimited, curDesAngularStates);
        ArmTelemetry.getInstance().setMeasured(curMeasState, curMeasAngularStates);

        // Save previous
        // A little wonky, because this needs to be prior to the 
        // offset, but including the soft limits.
        prevDesState = asl.applyLimit(curDesState);
    }

    private boolean isFaulted() {
        return false;
    }

    //Test-mode only for tuning
    public void testUpdate(){

        ArmEndEffectorState curMeasState = updateMeasState();

        var testDesState = new ArmAngularState();

        testDesState.shoulderAngleDeg = shoulderFG.getValue();
        testDesState.shoulderAnglularVel = shoulderFG.getValDeriv();
        testDesState.elbowAngleDeg = elbowFG.getValue();
        testDesState.elbowAngularVel = elbowFG.getValDeriv();

        mb.setCmd(testDesState);
        mb.update(curMeasAngularStates, shoulderFG.isEnabled());

        ms.setCmd(testDesState);
        ms.update(curMeasAngularStates,elbowFG.isEnabled());

        var curTestState = ArmKinematics.forward(testDesState);

        // Update telemetry
        ArmTelemetry.getInstance().setDesired(curTestState, testDesState);
        ArmTelemetry.getInstance().setMeasured(curMeasState, curMeasAngularStates);
        
    }

        public boolean isPathPlanning(){
        return pp.motionActive;
    }

    public double speedLimitFactorCalc(){
        return speedLimitMap.lookupVal(curDesState.x);
    }

    public boolean isSoftLimited(){
        return asl.isLimited() || mb.isAngleLimited || ms.isAngleLimited;
    }
    
}
