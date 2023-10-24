package org.firstinspires.ftc.teamcode.SubSystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

public class InTake extends SubsystemBase {
    DcMotor inTakeMotor;
    Servo intakeAngel;
    final double servoToIntakeRatio = 1;



    public InTake(DcMotor inTakeMotor, Servo intakeAngel){
        this.inTakeMotor = inTakeMotor;
        this.intakeAngel = intakeAngel;
    }

    public void setPosition(double pos){
        intakeAngel.setPosition(pos * servoToIntakeRatio);
    }

    public void setPower(double power){
        inTakeMotor.setPower(power);
    }

    public void stop(){
        setPower(0);
    }

}
