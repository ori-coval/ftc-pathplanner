package org.firstinspires.ftc.teamcode.SubSystems;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.geometry.Vector2d;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.Commands.drivetrain.TeleopDriveCommand;
import org.firstinspires.ftc.teamcode.Utils.Configuration;

import static org.firstinspires.ftc.teamcode.Utils.MathAccessories.*;


public class DriveTrain extends SubsystemBase {
    private final DcMotor motorFR, motorFL, motorBR, motorBL;
    private final BNO055IMU imu;
    private static final BNO055IMU.Parameters imuParameters = new BNO055IMU.Parameters() {{
        imuParameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
    }};


    public DriveTrain(HardwareMap hardwareMap, Gamepad gamepad1) {

        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(imuParameters);

        motorFR = hardwareMap.dcMotor.get(Configuration.DRIVE_TRAIN_FRONT_RIGHT);
        motorFL = hardwareMap.dcMotor.get(Configuration.DRIVE_TRAIN_FRONT_LEFT);
        motorBR = hardwareMap.dcMotor.get(Configuration.DRIVE_TRAIN_BACK_RIGHT);
        motorBL = hardwareMap.dcMotor.get(Configuration.DRIVE_TRAIN_BACK_LEFT);

        motorFL.setDirection(DcMotorSimple.Direction.REVERSE);
        motorBL.setDirection(DcMotorSimple.Direction.REVERSE);

        // make the resting state of the drive train be teleop drive
        this.setDefaultCommand(new TeleopDriveCommand(this, gamepad1));
    }



    public void drive(Vector2d direction, double turn){
        setPower(
                normalize(
                        calculateMecanumPowerRatio(direction, turn)
                )
        );
    }
    private void setPower(double[] power){
        motorFR.setPower(power[0]);
        motorFL.setPower(power[1]);
        motorBR.setPower(power[2]);
        motorBL.setPower(power[3]);
    }



    private double[] calculateMecanumPowerRatio(Vector2d direction, double turn){
        //                    { STRAIGHT }             { STRAFE }        { TURN }
        double FR_Power =   -direction.getY()   -   direction.getX()   -   turn   ;
        double FL_Power =   -direction.getY()   +   direction.getX()   +   turn   ;
        double BR_Power =    direction.getY()   +   direction.getX()   -   turn   ;
        double BL_Power =    direction.getY()   -   direction.getX()   +   turn   ;

        return new double[] { FR_Power, FL_Power, BR_Power, BL_Power };
    }

    public double getYawInDegrees(){
        return imu.getAngularOrientation().firstAngle;
    }
}