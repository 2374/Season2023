package frc.robot.subsystems;

import com.ctre.phoenix.led.CANdle;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.UsbCamera;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.util.GamePiece.GamePieceType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class ChassisSubsystem extends SubsystemBase {

    private CANdle candle;
    private GamePieceType gamePiece = GamePieceType.None; // is the robot working with a Cone or a Cube? default to None

    private String serialNumber = "unknown";
    private UsbCamera camera = CameraServer.startAutomaticCapture("Camera", 0);

    private static ChassisSubsystem instance;

    /**
     * Handles robot wide and generic systems
     */
    public ChassisSubsystem() {
        candle = new CANdle(Constants.CANDLE_CAN_ID, Constants.CAN_BUS_NAME_ROBORIO);
        candle.configLEDType(CANdle.LEDStripType.RGB);
        candle.configBrightnessScalar(0.5);
        instance = this;
        serialNumber = RobotController.getSerialNumber();
        System.out.println("SERIALNUMBER=" + serialNumber);
        // SmartDashboard.putData(camera); TODO fix this
    }

    /**
     * Makes the CANdle yellow
     */
    private void turnOnYellow() {
        System.out.println("yellow");
        candle.setLEDs(255, 255, 0);
    }

    /**
     * Makes the CANdle purple
     */
    private void turnOnPurple() {
        System.out.println("purple");
        candle.setLEDs(255, 0, 255);
    }

    /**
     * Turns off the candle.
     * This should only be called during testing as we should always be looking for
     * a CONE or a CUBE
     */
    private void turnOff() {
        System.out.println("off");
        candle.setLEDs(0, 0, 0);
    }

    /**
     * Gets the ChassisSubsystem instance. Makes a new one if not present.
     * 
     * @return The ChassisSubsystem
     */
    public static ChassisSubsystem getChassisInstance() {
        if (instance == null) {
            instance = new ChassisSubsystem();
        }
        return instance;
    }

    /**
     * Do we want to work with some object?
     * 
     * @return Do we want to work with some object?
     */
    public boolean getSomething() {
        return gamePiece != GamePieceType.None;
    }

    /**
     * Do we want to work with a cone?
     * 
     * @return TRUE if the robot is working with a CONE
     */
    public Boolean getWantACone() {
        return gamePiece == GamePieceType.Cone;
    }

    /**
     * Do we want to work with a cube?
     * 
     * @return TRUE if the robot is working with a CUBE
     */
    public Boolean getWantACube() {
        return gamePiece == GamePieceType.Cube;
    }

    public void setWantToggle() {
        if (gamePiece == GamePieceType.Cube) {
            setWantACone();
        } else {
            setWantACube();
        }
    }

    /**
     * Tell the robot it wants to work with a CONE
     */
    public void setWantACone() {
        turnOnYellow();
        gamePiece = GamePieceType.Cone;
    }

    /**
     * Tell the robot it wants to work with a CUBE
     */
    public void setWantACube() {
        turnOnPurple();
        gamePiece = GamePieceType.Cube;
    }

    /**
     * Tell the robot it doesn't want to work with game pieces
     */
    public void setWantNothing() {
        turnOff();
        gamePiece = GamePieceType.None;
    }

    /**
     * Is this swervee?
     * 
     * @return Is this swervee?
     */
    public Boolean isTestRobot() {
        Boolean result = serialNumber.equalsIgnoreCase(Constants.TEST_ROBORIO_SERIAL_NUMBER);
        System.out.println("RIOTEST=" + result);
        return result;
    }

}
