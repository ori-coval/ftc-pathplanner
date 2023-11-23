package org.firstinspires.ftc.teamcode.SubSystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.DcMotor;

public class Elevator extends SubsystemBase {

    //how do we get the height? do we just get a number or is some sort of conversion needed
    //like with angles and ticks?
    //also we need to decide how to control the elevator (servos, encoders, etc.)
    private DcMotor elevatorMotor;

    public Elevator(DcMotor elevatorMotor) {
        this.elevatorMotor = elevatorMotor;
    }

    public void setPower(double power) {
        elevatorMotor.setPower(power);
    }

}

