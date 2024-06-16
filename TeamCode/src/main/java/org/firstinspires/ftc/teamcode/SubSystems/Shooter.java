package org.firstinspires.ftc.teamcode.SubSystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.roboctopi.cuttlefishftcbridge.devices.CuttleMotor;

import org.firstinspires.ftc.teamcode.MMRobot;
import org.firstinspires.ftc.teamcode.Utils.Configuration;

public class Shooter extends SubsystemBase {

    CuttleMotor motor;

    public Shooter() {
        this.motor = MMRobot.getInstance().mmSystems.controlHub.getMotor(Configuration.SHOOTER);
        this.register();
    }

    public void setPower(double power) {
        motor.setPower(power);
    }

    @Override
    public void periodic() {
        MMRobot.getInstance().mmSystems.telemetry.addLine(String.valueOf(motor.power));
    }
}
