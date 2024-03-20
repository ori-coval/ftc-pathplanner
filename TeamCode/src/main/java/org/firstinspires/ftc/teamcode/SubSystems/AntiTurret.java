package org.firstinspires.ftc.teamcode.SubSystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Utils.Configuration;

public class AntiTurret extends SubsystemBase {
    private final Servo antiTurret;

    final private double M = .555/180;

    public AntiTurret(HardwareMap hardwareMap) {
        antiTurret = hardwareMap.servo.get(Configuration.ANTI_TURRET);
    }

    public void setAngle(double angle) {
        antiTurret.setPosition(M * angle + 0.045);
    }

    public double getPosition() {
        return antiTurret.getPosition();
    }
}
