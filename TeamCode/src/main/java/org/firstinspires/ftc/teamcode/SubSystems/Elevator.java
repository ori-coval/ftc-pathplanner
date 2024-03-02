package org.firstinspires.ftc.teamcode.SubSystems;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.Utils.Configuration;

@Config
public class Elevator extends SubsystemBase {
    private final DcMotor[] elevatorMotors = new DcMotor[2];
    private final DcMotor encoder;
    private int encoderOffset;
    private final DigitalChannel elevatorSwitch;
    private final double LEVELS = 3;
    private final double TEETH_PER_REV = 8;
    private final double CHAIN_LINK_DISTANCE = 0.8;
    private final double TICKS_PER_REV = 751.8;
    public static double kP = 0.22; //0.18
    public static double kI = 0;
    public static double kD = 0;
    public static double kG = 0.01;
    public static double kS = 0.02;

    private final PIDController pidController = new PIDController(kP,kI,kD);


    public Elevator(HardwareMap hardwareMap) {
        elevatorMotors[0] = hardwareMap.dcMotor.get(Configuration.ELEVATOR_RIGHT);
        elevatorMotors[1] = hardwareMap.dcMotor.get(Configuration.ELEVATOR_LEFT);
        elevatorMotors[0].setDirection(DcMotorSimple.Direction.REVERSE);
        encoder = elevatorMotors[0];
        encoder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        encoder.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER); //If doesn't work use RUN_USING_ENCODER
        elevatorSwitch = hardwareMap.digitalChannel.get(Configuration.ELEVATOR_SWITCH);

        elevatorMotors[0].setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        elevatorMotors[1].setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    public void setPower(double power) {
        for (DcMotor motor: elevatorMotors) {
            motor.setPower(power);
        }
    }

    public int getEncoderValue() {
        return encoder.getCurrentPosition() - encoderOffset;
    }

    public void resetEncoder() {
        encoderOffset = encoder.getCurrentPosition();
    }

    public double getHeight() {
        double motorRevs = getEncoderValue() / TICKS_PER_REV;
        double lengthPerRev = CHAIN_LINK_DISTANCE * TEETH_PER_REV;
        double pulledLength = lengthPerRev * motorRevs;
        return LEVELS * pulledLength;
    }

    public boolean getSwitchState() {
        return !elevatorSwitch.getState();
    }

    public void telemetry() {
        FtcDashboard.getInstance().getTelemetry().addData("Elevator Calculated power", pidController.calculate(getHeight()) + getKg());
        FtcDashboard.getInstance().getTelemetry().addData("Elevator Position", getHeight());
        FtcDashboard.getInstance().getTelemetry().addData("Elevator Target Position", pidController.getSetPoint());
        FtcDashboard.getInstance().getTelemetry().update();
    }

    @Override
    public void periodic() {
        telemetry();
    }

    public double getKg() {
        return kG;
    }

    public double getKs(){
        return kS;
    }

    public PIDController getPidController() {
        return pidController;
    }
}

