package org.firstinspires.ftc.teamcode.SubSystems;

import com.roboctopi.cuttlefish.utils.Direction;

import org.firstinspires.ftc.teamcode.CuttlefishFTCBridge.src.devices.MMMotor;
import org.firstinspires.ftc.teamcode.CuttlefishFTCBridge.src.devices.MMServo;
import org.firstinspires.ftc.teamcode.MMLib.MMPowerSubsystem;
import org.firstinspires.ftc.teamcode.MMRobot;
import org.firstinspires.ftc.teamcode.Utils.Configuration;

public class Shooter extends MMPowerSubsystem<Double> {

    MMRobot mmRobot = MMRobot.getInstance();

    MMMotor motor1;
    MMMotor motor2;
    MMServo servo;

    public Shooter() {
        this.motor1 = mmRobot.mmSystems.controlHub.getMotor(Configuration.SHOOTER1);
        this.motor2 = mmRobot.mmSystems.controlHub.getMotor(Configuration.SHOOTER2);
        motor1.setDirection(Direction.REVERSE);
        this.servo = mmRobot.mmSystems.controlHub.getServo(Configuration.SHOOTER_SERVO);
    }

    @Override
    public void setPower(Double power) {
        motor1.setPower(power);
        motor2.setPower(power);
    }

    public void setPosition(double pos) {
        servo.setPosition(pos);
    }

    @Override
    public void periodic() {
        MMRobot.getInstance().mmSystems.telemetry.addData("Power", String.valueOf(motor1.getPower()));
    }
}
