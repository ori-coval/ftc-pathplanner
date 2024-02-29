package org.firstinspires.ftc.teamcode.SubSystems;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.geometry.Vector2d;
import com.qualcomm.hardware.bosch.BHI260IMU;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcontroller.external.samples.RobotHardware;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.MotionDetection;
import org.firstinspires.ftc.teamcode.Utils.Configuration;

public class DriveTrain extends SubsystemBase {
    private final DcMotor motorFR;
    private final DcMotor motorFL;
    private final DcMotor motorBL;
    private final DcMotor motorBR;
    private final BNO055IMU imu;
    private double yawOffset = 0;

    public DriveTrain(HardwareMap hardwareMap) {
        imu = hardwareMap.get(BNO055IMU.class, Configuration.IMU);
        BNO055IMU.Parameters imuParameters = new BNO055IMU.Parameters();
        imuParameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        imu.initialize(imuParameters);

        motorFR = hardwareMap.dcMotor.get(Configuration.DRIVE_TRAIN_FRONT_RIGHT);
        motorFL = hardwareMap.dcMotor.get(Configuration.DRIVE_TRAIN_FRONT_LEFT);
        motorBR = hardwareMap.dcMotor.get(Configuration.DRIVE_TRAIN_BACK_RIGHT);
        motorBL = hardwareMap.dcMotor.get(Configuration.DRIVE_TRAIN_BACK_LEFT);
        motorFL.setDirection(DcMotorSimple.Direction.REVERSE);
        motorBL.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    public DriveTrain(HardwareMap hardwareMap, double lastAngle){
        this(hardwareMap);
        setYaw(lastAngle);
    }

    public double getYawInDegrees() {
        return imu.getAngularOrientation().firstAngle + yawOffset;
    }

    public void setYaw(double newYaw) {
        yawOffset = newYaw - imu.getAngularOrientation().firstAngle; //TODO need to be checked
    }

    public void resetYaw() {
        setYaw(0);
    }
    public double[] calculationOfPowerRatio(double x, double y , double turn) {
        //                     {STRAIGHT}                 {STRAFE}                  {TURN}
        double FL_Power =           y           +            x           +          turn         ;
        double BL_Power =           y            -            x           +          turn         ;
        double FR_Power =           y           -            x           -          turn         ;
        double BR_Power =           y            +            x           -          turn         ;
        return new double[]{FL_Power,BL_Power,FR_Power,BR_Power};
    }
    public static double[] normalize(double[] ratiopower) {
        double[] power = ratiopower;
        double highestAbsulutNum = Math.max(
                Math.max(Math.abs(ratiopower[2]),Math.abs(ratiopower[3])),
                Math.max(Math.abs(ratiopower[0]),Math.abs(ratiopower[1])));
        if (highestAbsulutNum < 1){return ratiopower;}
        for (int i = 0; i < 4; i++) {
            power[i] = ratiopower[i] / highestAbsulutNum;
        }
        return power;
    }
    private void setMotorPower(double[] normalize) {
        motorFL.setPower(normalize[0]);
        motorBL.setPower(normalize[1]);
        motorFR.setPower(normalize[2]);
        motorBR.setPower(normalize[3]);
    }
    public void drive(double x,double y, double turn) {
        setMotorPower(normalize(calculationOfPowerRatio(x, y, turn)));
    }

    public void fieldOrientedDrive(double x, double y, double turn) {
        Vector2d joyStickDirection = new Vector2d(x,y);
        Vector2d fieldOrientedVector = joyStickDirection.rotateBy(-getYawInDegrees());
        drive(fieldOrientedVector.getX(),fieldOrientedVector.getY(),turn);
    }


}