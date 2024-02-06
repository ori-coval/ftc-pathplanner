package org.firstinspires.ftc.teamcode.SubSystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Extender extends SubsystemBase {
    private final Servo linearServo;

    private Position curretPosition;

    public enum Position {
        CLOSED(0.0675), MID_WAY(0.29625), OPEN(0.5925);
        private final double servoPosition;
        Position(double position){
            this.servoPosition = position;
        }
    }

    public Extender (HardwareMap hardwareMap){
        linearServo = hardwareMap.servo.get("extender");
        linearServo.setDirection(Servo.Direction.REVERSE);
    }

    public double getPos() {
        return linearServo.getPosition();
    }

    public void setPos(double pos) {
        linearServo.setPosition(pos);
    }

    public void setPosition(Position position) {
        linearServo.setPosition(position.servoPosition);
        curretPosition = position;
    }

    public Position getCurretPosition() {
        return curretPosition;
    }
}
