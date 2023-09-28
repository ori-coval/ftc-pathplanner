package org.firstinspires.ftc.teamcode;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;

public class DriveTrain extends SubsystemBase {

    DcMotor motor_FR;
    DcMotor motor_FL;
    DcMotor motor_BL;
    DcMotor motor_BR;
    Gamepad gamepad;

    public DriveTrain(DcMotor motor_BL, DcMotor motor_BR, DcMotor motor_FL, DcMotor motor_FR,Gamepad gamepad1) {
        this.motor_BL = motor_BL;
        this.motor_FL = motor_FL;
        this.motor_FR = motor_FR;
        this.motor_BR = motor_BR;
        this.gamepad = gamepad1;
    }
    public static double[]CalculationOfPowerRatio(Gamepad gamepad1){
        //                     {STRAIGHT}                 {STRAFE}                {TURN}
        double FR_Power = gamepad1.left_stick_y + gamepad1.left_stick_x - gamepad1.right_stick_x;
        double FL_Power = gamepad1.left_stick_y - gamepad1.left_stick_x + gamepad1.right_stick_x;
        double BR_Power = gamepad1.left_stick_y - gamepad1.left_stick_x - gamepad1.right_stick_x;
        double BL_Power = gamepad1.left_stick_y + gamepad1.left_stick_x + gamepad1.right_stick_x;
        double[] PowerRatio = {FR_Power,FL_Power,BR_Power,BL_Power};
        return PowerRatio;
    }


}

