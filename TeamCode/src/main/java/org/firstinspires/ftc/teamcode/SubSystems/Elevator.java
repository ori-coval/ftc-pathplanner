package org.firstinspires.ftc.teamcode.SubSystems;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

@Config
public class Elevator extends SubsystemBase {
    private DcMotor[] elevatorMotors = new DcMotor[3];
    private DcMotor encoder;
    private final double LEVELS = 3;
    private final double TEETH_PER_REV = 8;
    private final double CHAIN_LINK_DISTANCE = 0.8;
    private final double TICKS_PER_REV = 384.5;
    public static double kP = 0.16; //0.165
    public static double kI = 0;
    public static double kD = 0;
    public static double kF = 0.15;
    private PIDController pidController = new PIDController(kP,kI,kD);


    public Elevator(HardwareMap hardwareMap) {
        elevatorMotors[0] = hardwareMap.dcMotor.get("elevatorLow");
        elevatorMotors[1] = hardwareMap.dcMotor.get("elevatorMid");
        elevatorMotors[2] = hardwareMap.dcMotor.get("elevatorUp");
        elevatorMotors[1].setDirection(DcMotorSimple.Direction.REVERSE);
        encoder = elevatorMotors[0];
        encoder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        encoder.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void setPower(double power) {
        for (DcMotor motor: elevatorMotors) {
            motor.setPower(power);
        }
    }

    public double getHeight(){
        double motorRevs = encoder.getCurrentPosition() / TICKS_PER_REV;
        double lengthPerRev = CHAIN_LINK_DISTANCE * TEETH_PER_REV;
        double pulledLength = lengthPerRev * motorRevs;
        return LEVELS * pulledLength;
    }

    public void telemetry() {
        FtcDashboard.getInstance().getTelemetry().addData("Elevator Calculated power", pidController.calculate(getHeight()) + getKF());
        FtcDashboard.getInstance().getTelemetry().addData("Elevator Position", getHeight());
        FtcDashboard.getInstance().getTelemetry().addData("Elevator Target Position", pidController.getSetPoint());
        FtcDashboard.getInstance().getTelemetry().update();
    }

    @Override
    public void periodic() {
        telemetry();
    }

    public double getKF() {
        return kF;
    }

    public PIDController getPidController() {
        return pidController;
    }
}

