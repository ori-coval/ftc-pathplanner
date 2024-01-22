package org.firstinspires.ftc.teamcode.SubSystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.Servo;

public class AntiTurret extends SubsystemBase {
    private Servo antiTurret;

    public AntiTurret(Servo servo) {
        this.antiTurret = servo;
    }

    public void setPosition(double angle) {
        antiTurret.setPosition(angle / 360);
    }

    public double getPosition() {
        return antiTurret.getPosition();
    }
}
