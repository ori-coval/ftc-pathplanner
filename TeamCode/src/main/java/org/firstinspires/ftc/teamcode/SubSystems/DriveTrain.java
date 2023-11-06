package org.firstinspires.ftc.teamcode.SubSystems;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.geometry.Vector2d;
import com.qualcomm.hardware.bosch.BHI260IMU;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcontroller.external.samples.RobotHardware;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.MotionDetection;

public class DriveTrain extends SubsystemBase {
    private DcMotor motorFR;
    private DcMotor motorFL;
    private DcMotor motorBL;
    private DcMotor motorBR;
    private BHI260IMU imu;

    public DriveTrain(DcMotor motor_BL, DcMotor motor_BR, DcMotor motor_FL, DcMotor motor_FR, BHI260IMU imu) {
        this.motorBL = motor_BL;
        this.motorFL = motor_FL;
        this.motorFR = motor_FR;
        this.motorBR = motor_BR;
        this.motorFL.setDirection(DcMotorSimple.Direction.REVERSE);
        this.motorBL.setDirection(DcMotorSimple.Direction.REVERSE);
        this.imu = imu;
    }

    public double getYaw(){
        return imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
    }
    public double[] calculationOfPowerRatio(double x, double y , double turn){
        //                     {STRAIGHT}                 {STRAFE}                  {TURN}
        double FR_Power =           y            +            x           -          turn         ;
        double FL_Power =           y            -            x           +          turn         ;
        double BR_Power =           y            -            x           -          turn         ;
        double BL_Power =           y            +            x           +          turn         ;
        double[] PowerRatio = {FR_Power,FL_Power,BR_Power,BL_Power};
        return PowerRatio;
    }
    public static double[] normalize(double[] ratiopower){
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

    public void fieldOrientedDrive(double x, double y, double turn){

    }


}

