package org.firstinspires.ftc.teamcode.Libraries.MMLib.Examples.Subsystems;

import com.roboctopi.cuttlefish.utils.Direction;

import org.firstinspires.ftc.teamcode.Libraries.CuttlefishFTCBridge.src.devices.CuttleMotor;
import org.firstinspires.ftc.teamcode.Libraries.CuttlefishFTCBridge.src.devices.CuttleServo;
import org.firstinspires.ftc.teamcode.Libraries.MMLib.SubsystemStructure.MMPowerPositionSubsystem;
import org.firstinspires.ftc.teamcode.MMRobot;
import org.firstinspires.ftc.teamcode.Utils.Configuration;

public class Shooter extends MMPowerPositionSubsystem<Double, Double> {

    MMRobot mmRobot = MMRobot.getInstance();

    CuttleMotor motor1;
    CuttleMotor motor2;
    CuttleServo servo;

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

    @Override
    public void setPosition(Double pos) {
        servo.setPosition(pos);
    }

    @Override
    public void periodic() {
        MMRobot.getInstance().mmSystems.telemetry.addData("Power", String.valueOf(motor1.getPower()));
    }
}
