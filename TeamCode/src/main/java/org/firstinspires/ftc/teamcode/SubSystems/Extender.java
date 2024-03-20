package org.firstinspires.ftc.teamcode.SubSystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Utils.Configuration;

public class Extender extends SubsystemBase {
    private final Servo linearServo;
    private Position curretPosition;

    public enum Position {
        CLOSED(0), AUTONOMOUS_PURPLE(0.15), TOP_CLOSE(0.2), BOTTOM_FRONT(0.36), MID_WAY(0.40), OPEN(0.56);
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

    public Position getExtenderPosition() {
        return curretPosition;
    }

    public double getServoPosition() {
        return linearServo.getPosition();
    }
}
