package org.firstinspires.ftc.teamcode.SubSystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

public class InTake extends SubsystemBase {
    private DcMotor inTakeMotor;
    private Servo inTakeAngle;
    public final double COLLECT_POWER = 0.8;
    public final double EJECT_POWER = -0.9;



    public InTake(DcMotor inTakeMotor, Servo inTakeAngle){
        this.inTakeMotor = inTakeMotor;
        this.inTakeAngle = inTakeAngle;
    }

    public void setPower(double power){
        inTakeMotor.setPower(power);
    }

    public void stop(){
        setPower(0);
    }

    public void setPosition(double position){
        inTakeAngle.setPosition(position);
    }
    public double getPosition(){return inTakeAngle.getPosition();}

}
