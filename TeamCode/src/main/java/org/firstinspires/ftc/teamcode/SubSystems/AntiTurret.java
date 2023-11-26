package org.firstinspires.ftc.teamcode.SubSystems;

import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoImplEx;

import org.checkerframework.checker.units.qual.A;

public class AntiTurret extends SubsystemBase {
    private Servo servo;

    public AntiTurret(Servo servo) {
        this.servo = servo;
    }

    public void setPosition(double angle) {
        servo.setPosition(angle / 360);
    }

    public double getPosition() {
        return servo.getPosition();
    }
}
