package org.firstinspires.ftc.teamcode.SubSystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.roboctopi.cuttlefish.utils.Direction;

import org.firstinspires.ftc.teamcode.CuttlefishFTCBridge.src.devices.MMMotor;
import org.firstinspires.ftc.teamcode.MMRobot;
import org.firstinspires.ftc.teamcode.Utils.Configuration;

public class Shooter extends SubsystemBase {

    MMMotor motor;

    public Shooter() {
        this.motor = MMRobot.getInstance().mmSystems.controlHub.getMotor(Configuration.SHOOTER);
        motor.setDirection(Direction.REVERSE);
        this.register();
    }

    public void setPower(double power) {
        motor.setPower(power);
    }

    @Override
    public void periodic() {
        MMRobot.getInstance().mmSystems.telemetry.addData("Power", String.valueOf(motor.getPower()));
    }
}
