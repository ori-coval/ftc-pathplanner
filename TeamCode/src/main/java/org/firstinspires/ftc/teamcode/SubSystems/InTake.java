package org.firstinspires.ftc.teamcode.SubSystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

public class InTake extends SubsystemBase {
    private DcMotor inTakeMotor;
    private Servo inTakeAngle;
    public final double COLLECT_POWER = 0.8;
    public final double EJECT_POWER = -0.9;
    public final double STACK_POSITION[] = {0.49, 0.52, 0.55, 0.58, 0.62};
    private double target = STACK_POSITION[5];


    // the first value to 5 piksel is 0.62
    // the second value to 4 piksel is 0.58
    // the third value to 3 piksel is 0.55
    // the fourth value to 2 piksel is 0.52
    // the fifth value to 1 piksel is 0.49
    // כשהאיסוף למעלה 0.67

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

    @Override
    public void periodic() {

    }
}
