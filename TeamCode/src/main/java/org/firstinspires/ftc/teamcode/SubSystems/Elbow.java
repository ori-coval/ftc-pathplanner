package org.firstinspires.ftc.teamcode.SubSystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.hardware.DcMotor;

public class Elbow extends SubsystemBase {
    private DcMotor elbowMotor;
    private final double elbowTicksPerRotation = 1;
    private PIDController pidController = new PIDController(0,0,0);
    public Elbow (DcMotor elbowMotor){
        this.elbowMotor = elbowMotor;
    }
    public void setPower(double power){
        elbowMotor.setPower(power);
    }
    public void stop(){
        setPower(0);
    }
    public double getAngle(){
        int ticks = elbowMotor.getCurrentPosition();
        return (ticks/elbowTicksPerRotation)*360;
    }

    public PIDController getPidController() {
        return pidController;
    }
}
