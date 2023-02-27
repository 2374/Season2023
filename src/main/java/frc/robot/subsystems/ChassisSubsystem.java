package frc.robot.subsystems;

import com.ctre.phoenix.led.CANdle;

import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.util.GamePiece.GamePieceType;

public class ChassisSubsystem extends SubsystemBase {

    private CANdle candle;
    private GamePieceType gamePiece = GamePieceType.None; // is the robot working with a Cone or a Cube? default to None
                                                
    private String serialNumber = "unknown";

    private static ChassisSubsystem instance;

    public ChassisSubsystem() {
        candle = new CANdle(Constants.CANDLE_CAN_ID, Constants.CAN_BUS_NAME_ROBORIO);
        candle.configLEDType(CANdle.LEDStripType.RGB);
        candle.configBrightnessScalar(0.5);
        instance = this;
        serialNumber = RobotController.getSerialNumber();
        System.out.println("SERIALNUMBER=" + serialNumber);
    }

    private void turnOnYellow() {
        System.out.println("yellow");
        candle.setLEDs(255, 255, 0);
    }

    private void turnOnPurple() {
        System.out.println("purple");
        candle.setLEDs(255, 0, 255);
    }

    // this should only be called during testing as we should always be looking for
    // a CONE or a CUBE
    private void turnOff() {
        System.out.println("off");
        candle.setLEDs(0, 0, 0);
    }

    public static ChassisSubsystem getChassisInstance() {
        if (instance == null) {
            instance = new ChassisSubsystem();
        }
        return instance;
    }

    // Do we want to work with some object?
    public boolean getSomething() {
        return gamePiece != GamePieceType.None;
    }

    // return TRUE if the robot is working with a CONE
    public Boolean getWantACone() {
        return gamePiece == GamePieceType.Cone;
    }

    // return TRUE if the robot is working with a CUBE
    public Boolean getWantACube() {
        return gamePiece == GamePieceType.Cube;
    }

    // tell the robot it wants to work with a CONE
    public void setWantACone() {
        turnOnYellow();
        gamePiece = GamePieceType.Cone;
    }

    // tell the robot it wants to work with a CUBE
    public void setWantACube() {
        turnOnPurple();
        gamePiece = GamePieceType.Cube;
    }

    // tell the robot it doesn't want to work with game pieces
    public void setWantNothing() {
        turnOff();
        gamePiece = GamePieceType.None;
    }

    // Is this Swervee?
    public Boolean isTestRobot() {
        Boolean result = serialNumber.equalsIgnoreCase(Constants.TEST_ROBORIO_SERIAL_NUMBER);
        System.out.println("RIOTEST=" + result);
        return result;
    }

}
