package org.firstinspires.ftc.teamcode.SubSystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

public class InTake extends SubsystemBase {
    DcMotor inTakeMotor;
    final double servoToIntakeRatio = 1;

    final double tolerance = 2.5;


    public InTake(DcMotor inTakeMotor){
        this.inTakeMotor = inTakeMotor;
    }
    public void setPower(double power){
        inTakeMotor.setPower(power);
    }

    public void stop(){
        setPower(0);
    }

}
