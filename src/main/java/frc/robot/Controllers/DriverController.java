package frc.robot.Controllers;

import edu.wpi.first.wpilibj.XboxController;

public class DriverController extends XboxController{
/**
 * Controller Map:
 * 
 * Left Stick X:
 * Left Stick Y: Drive Throttle
 * Left Stick Button:
 * 
 * Right Stick X: Drive Turn
 * Right Stick Y:
 * Right Stick Button:
 * 
 * Left Bumper:
 * Right Bumper: 
 * 
 * Left Trigger: 
 * Right Trigger: 
 * 
 * Button A:
 * Button B: 
 * Button X:
 * Button Y:
 * 
 * Button Select:
 * Button Start:
 * 
 * POV 0: 
 * POV 45:
 * POV 90: 
 * POV 135:
 * POV 180: 
 * POV 225:
 * POV 270:
 */

    public DriverController(int port) {
        super(port);
    }

    public double getSpeed() {
        return getLeftY();
    }

    public double getRotation() {
        return getRightX();
    }

}
