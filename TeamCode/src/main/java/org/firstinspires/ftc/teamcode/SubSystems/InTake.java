package org.firstinspires.ftc.teamcode.SubSystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.DcMotor;

public class InTake extends SubsystemBase {

    DcMotor inTakeMotor;

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
