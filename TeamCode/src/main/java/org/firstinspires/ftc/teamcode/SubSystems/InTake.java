package org.firstinspires.ftc.teamcode.SubSystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

public class InTake extends SubsystemBase {
    private DcMotor inTakeMotor;
    public final double COLLECT_POWER = 0.8;
    public final double EJECT_POWER = -0.9;


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
