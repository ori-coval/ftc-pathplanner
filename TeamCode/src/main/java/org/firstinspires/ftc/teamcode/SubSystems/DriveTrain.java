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

import org.apache.commons.math3.linear.MatrixUtils;
import org.apache.commons.math3.linear.RealMatrix;
import org.apache.commons.math3.linear.RealVector;
import org.firstinspires.ftc.robotcontroller.external.samples.RobotHardware;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.MotionDetection;
import org.firstinspires.ftc.teamcode.Utils.Configuration;

public class DriveTrain extends SubsystemBase {

    final double[][] transformationMatrix = {
            {1, 1, 1}, //frontLeft
            {-1, 1, 1}, //backLeft
            {-1, 1, -1}, //frontRight
            {1, 1, -1} //backRight
    };

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

    private double[] joystickToPower(double x, double y, double yaw) {

        RealVector joystickVector = MatrixUtils.createRealVector(new double[] {
                x,
                y,
                yaw
        });

        RealMatrix matrixT = MatrixUtils.createRealMatrix(transformationMatrix);

        RealVector powerVector = matrixT.operate(joystickVector); //p = Tv

        double[] powerArray = powerVector.toArray();

        for(int i = 0; i < powerArray.length; i++) {
            powerArray[i] = powerArray[i] / Math.max(Math.abs(x) + Math.abs(y) + Math.abs(yaw), 1);
        }

        return powerArray;

    }



    private void setMotorPower(double[] power) {
        motorFL.setPower(power[0]);
        motorBL.setPower(power[1]);
        motorFR.setPower(power[2]);
        motorBR.setPower(power[3]);
    }
    public void drive(double x, double y, double yaw) {
        setMotorPower(joystickToPower(x, y, yaw));
    }


    public void fieldOrientedDrive(double x, double y, double yaw) {
        Vector2d joystickDirection = new Vector2d(x, y);
        Vector2d fieldOrientedVector = joystickDirection.rotateBy(-getYawInDegrees());
        drive(fieldOrientedVector.getX(), fieldOrientedVector.getY(), yaw);
    }

    public double getYawInDegrees() {
        return imu.getAngularOrientation().firstAngle + yawOffset;
    }

    public void setYaw(double newYaw) {
        yawOffset = newYaw - imu.getAngularOrientation().firstAngle;
    }

    public void resetYaw() {
        setYaw(0);
    }

}