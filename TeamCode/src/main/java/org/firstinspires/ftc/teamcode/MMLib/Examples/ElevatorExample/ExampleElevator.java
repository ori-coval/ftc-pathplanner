package org.firstinspires.ftc.teamcode.MMLib.Examples.ElevatorExample;

import org.firstinspires.ftc.teamcode.CuttlefishFTCBridge.src.devices.MMEncoder;
import org.firstinspires.ftc.teamcode.CuttlefishFTCBridge.src.devices.MMMotor;
import org.firstinspires.ftc.teamcode.MMLib.PID.MMPIDSubsystem;
import org.firstinspires.ftc.teamcode.MMRobot;
import org.firstinspires.ftc.teamcode.Utils.Configuration;

public class ExampleElevator extends MMPIDSubsystem {

    //hardware
    private final MMMotor motorLeft;
    private final MMMotor motorRight;
    private final MMEncoder encoder;

    //control
    private static final double kP = 0;
    private static final double kI = 0;
    private static final double kD = 0;
    private static final double tolerance = 0;

    //constants
    private final double TICKS_PER_REV = 384.5;
    private final double SPROCKET_PITCH_RADIUS = 1.05;
    private final int ELEVATOR_LEVELS = 3;

    public ExampleElevator() {
        super(kP, kI, kD, tolerance);
        motorLeft = new MMMotor(MMRobot.getInstance().mmSystems.expansionHub, Configuration.ELEVATOR_LEFT);
        motorRight = new MMMotor(MMRobot.getInstance().mmSystems.expansionHub, Configuration.ELEVATOR_RIGHT);
        encoder = new MMEncoder(MMRobot.getInstance().mmSystems.expansionHub, Configuration.ELEVATOR_ENCODER, TICKS_PER_REV);
    }

    @Override
    public void setPower(Double power) {
        motorRight.setPower(power);
        motorLeft.setPower(power);
    }

    /**
     * get the current value of the elevator.<p>
     * note that in every elevator it's different,</p>
     * this is something you are going to have to find urself.
     * @return the height in cm.
     */
    @Override
    public double getCurrentValue() {
        double revs = encoder.getCounts() / TICKS_PER_REV;
        double chainLengthPerRev = 2 * Math.PI * SPROCKET_PITCH_RADIUS;
        return revs * chainLengthPerRev * ELEVATOR_LEVELS;
    }

    @Override
    public void stop() {
        setPower((double) 0);
    }
}
