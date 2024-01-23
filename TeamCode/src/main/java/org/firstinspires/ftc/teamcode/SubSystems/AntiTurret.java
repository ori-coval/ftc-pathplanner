package org.firstinspires.ftc.teamcode.SubSystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.Servo;

import java.util.function.DoubleSupplier;

public class AntiTurret extends SubsystemBase {
    private Servo antiTurret;

    public AntiTurret(Servo servo) {
        this.antiTurret = servo;
    }

    public void setPos(double pos) {
        antiTurret.setPosition(pos);
    }

    public double getPosition() {
        return antiTurret.getPosition();
    }
}
