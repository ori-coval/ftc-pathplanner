package org.firstinspires.ftc.teamcode.SubSystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

public class InTake extends SubsystemBase {
    DcMotor inTakeMotor;
    Servo intakeAngel;
    final double servoToIntakeRatio = 1;

    final double tolorens = 2.5;


    public InTake(DcMotor inTakeMotor, Servo intakeAngel){
        this.inTakeMotor = inTakeMotor;
        this.intakeAngel = intakeAngel;
    }

    public double getPosition(){
        return intakeAngel.getPosition() / servoToIntakeRatio;
    }

    public boolean atTarget(double target){
        return Math.abs(getPosition() - target) < tolorens;
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
