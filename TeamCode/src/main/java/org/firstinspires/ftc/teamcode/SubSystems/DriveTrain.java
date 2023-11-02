package org.firstinspires.ftc.teamcode.SubSystems;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.robotcontroller.external.samples.RobotHardware;

public class DriveTrain extends SubsystemBase {

    DcMotor motorFR;
    DcMotor motorFL;
    DcMotor motorBL;
    DcMotor motorBR;

    public DriveTrain(DcMotor motor_BL, DcMotor motor_BR, DcMotor motor_FL, DcMotor motor_FR) {
        this.motorBL = motor_BL;
        this.motorFL = motor_FL;
        this.motorFR = motor_FR;
        this.motorBR = motor_BR;
    }
    private double[] calculationOfPowerRatio(double x, double y , double turn){
        //                     {STRAIGHT}                 {STRAFE}                {TURN}
        double FR_Power =           y            +            x           -        turn;
        double FL_Power =           y            -            x           +        turn;
        double BR_Power =           y            -            x           -        turn;
        double BL_Power =           y            +            x           +        turn;
        double[] PowerRatio = {FR_Power,FL_Power,BR_Power,BL_Power};
        return PowerRatio;
    }
    private static double[] normalize(double[] ratiopower){
        double[] power = ratiopower;
        double highestAbsulutNum = Math.max(Math.max(ratiopower[2],ratiopower[3]),Math.max(ratiopower[0],ratiopower[1]));
        for (int i = 0; i < 4; i++) {
            power[i] = ratiopower[i] / highestAbsulutNum;
        }
        return power;
    }
    private void setMotorPower(double[] normalize){
        motorFR.setPower(normalize[0]);
        motorFL.setPower(normalize[1]);
        motorBR.setPower(normalize[2]);
        motorBL.setPower(normalize[3]);
    }
    public void drive(double x,double y, double turn){
        setMotorPower(normalize(calculationOfPowerRatio(x, y, turn)));
    }

}

