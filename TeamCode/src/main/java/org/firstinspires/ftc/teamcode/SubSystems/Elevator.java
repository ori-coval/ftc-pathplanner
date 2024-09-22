package org.firstinspires.ftc.teamcode.SubSystems;

import org.firstinspires.ftc.teamcode.Libraries.CuttlefishFTCBridge.src.devices.CuttleEncoder;
import org.firstinspires.ftc.teamcode.Libraries.CuttlefishFTCBridge.src.devices.CuttleMotor;
import org.firstinspires.ftc.teamcode.Libraries.MMLib.PID.MMPIDSubsystem;
import org.firstinspires.ftc.teamcode.MMRobot;
import org.firstinspires.ftc.teamcode.Utils.Configuration;

public class Elevator extends MMPIDSubsystem {

    MMRobot robot = MMRobot.getInstance();

    CuttleMotor motorRight;
    CuttleMotor motorLeft;
    CuttleEncoder motorLeftEncoder;

    final double TicksForRotation = 0;
    final double LEVELS = 4;
    final double SPROCKET_PERIMETER = 20.9;

    static final double KI = 0;
    static final double KP = 0;
    static final double KD = 0;
    static final double TOLERANCE = 0;


    public Elevator() {
        super(KP,KI,KD,TOLERANCE);
        this.motorRight = new CuttleMotor(robot.mmSystems.expansionHub,(Configuration.ELEVATOR_RIGHT));
        this.motorLeft = new CuttleMotor(robot.mmSystems.expansionHub,(Configuration.ELEVATOR_LEFT));
        this.motorLeftEncoder = new CuttleEncoder(robot.mmSystems.expansionHub,Configuration.ELEVATOR_ENCODER,1);
    }

    public double getHeight(){
        return motorLeftEncoder.getCounts()/TicksForRotation * (LEVELS * SPROCKET_PERIMETER);
    }

    @Override
    public double getCurrentValue() {
        return getHeight();
    }

    @Override
    public void setPower(Double power) {
        motorLeft.setPower(power);
        motorRight.setPower(power);
    }
}
