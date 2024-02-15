package org.firstinspires.ftc.teamcode.SubSystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Utils.Configuration;

public class Extender extends SubsystemBase {
    private final Servo linearServo;
    private Position curretPosition;

    public enum Position {
        CLOSED(0.0675), CLOSED_INTAKE(0.15), MID_WAY(0.28), OPEN(0.56);
        private final double servoPosition;
        Position(double position){
            this.servoPosition = position;
        }
        public double getServoPositionAsDouble() {
            return servoPosition;
        }
    }

    public Extender (HardwareMap hardwareMap){
        linearServo = hardwareMap.servo.get(Configuration.EXTENDER);
        linearServo.setDirection(Servo.Direction.REVERSE);
    }

    public void setPosition(Position position) {
        linearServo.setPosition(position.servoPosition);
        curretPosition = position;
    }

    public Position getPosition() {
        return curretPosition;
    }
}
