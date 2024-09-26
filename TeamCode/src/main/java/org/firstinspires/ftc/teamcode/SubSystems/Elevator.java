package org.firstinspires.ftc.teamcode.SubSystems;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;

import org.firstinspires.ftc.teamcode.Libraries.CuttlefishFTCBridge.src.devices.CuttleEncoder;
import org.firstinspires.ftc.teamcode.Libraries.CuttlefishFTCBridge.src.devices.CuttleMotor;
import org.firstinspires.ftc.teamcode.Libraries.MMLib.PID.MMPIDSubsystem;
import org.firstinspires.ftc.teamcode.MMRobot;
import org.firstinspires.ftc.teamcode.Utils.Configuration;

@Config
public class Elevator extends MMPIDSubsystem {

    MMRobot robot = MMRobot.getInstance();

    CuttleMotor motorRight;
    CuttleMotor motorLeft;
    public CuttleEncoder motorLeftEncoder;

    final double TICKS_PER_REV = 384.5;
    final double LEVELS = 4;
    final double SPROCKET_PERIMETER = 6.56592;

    public static final double KP = 0.03;
    public static final double KI = 0;
    public static final double KD = 0;
    static final double TOLERANCE = 3;


    public Elevator() {
        super(KP,KI,KD,TOLERANCE);
        this.motorRight = new CuttleMotor(robot.mmSystems.expansionHub,(Configuration.ELEVATOR_RIGHT));
        this.motorLeft = new CuttleMotor(robot.mmSystems.expansionHub,(Configuration.ELEVATOR_LEFT));
        this.motorLeftEncoder = new CuttleEncoder(robot.mmSystems.expansionHub,Configuration.ELEVATOR_ENCODER,TICKS_PER_REV);
    }

    public double getHeight(){
        return ((motorLeftEncoder.getCounts() / TICKS_PER_REV)*SPROCKET_PERIMETER*LEVELS) ;
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

    public void updateToDashboard(){
//        FtcDashboard.getInstance().getTelemetry().addData("motorLeft", ()->motorLeft.getPower());
//        FtcDashboard.getInstance().getTelemetry().addData("motorRight", ()->motorRight.getPower());
        FtcDashboard.getInstance().getTelemetry().addData("height",getHeight());
        FtcDashboard.getInstance().getTelemetry().update();
    }

    @Override
    public double getFeedForwardPower() {
        return 0;
    }
}
