package org.firstinspires.ftc.teamcode;

import android.graphics.Path;

import com.arcrobotics.ftclib.command.Robot;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.SubSystems.Shooter;
import org.firstinspires.ftc.teamcode.SubSystems.ShooterPID;
import org.firstinspires.ftc.teamcode.Utils.AllianceColor;
import org.firstinspires.ftc.teamcode.Utils.AllianceSide;
import org.firstinspires.ftc.teamcode.Utils.OpModeType;

public class MMRobot extends Robot {

    private static MMRobot instance;

    public static synchronized MMRobot getInstance() {
        if (instance == null) {
            instance = new MMRobot();
        }
        return instance;
    }

    public void resetRobot() {
        instance = null;
    }

    //Systems

    public MMRobotParams mmSystems = new MMRobotParams();


    public void init(OpModeType type, HardwareMap hardwareMap, Gamepad gamepad1, Gamepad gamepad2, Telemetry telemetry) {
        initializeAttributes(type, hardwareMap, gamepad1, gamepad2, telemetry);
        initializeSystems(type);
    }

    public void init(OpModeType type, AllianceColor allianceColor, HardwareMap hardwareMap, Gamepad gamepad1, Gamepad gamepad2, Telemetry telemetry) {
        init(type, hardwareMap, gamepad1, gamepad2, telemetry);
        mmSystems.allianceColor = allianceColor;
    }

    public void init(OpModeType type, AllianceColor allianceColor, AllianceSide robotSide, HardwareMap hardwareMap, Gamepad gamepad1, Gamepad gamepad2, Telemetry telemetry) {
        init(type, allianceColor, hardwareMap, gamepad1, gamepad2, telemetry);
        mmSystems.robotSide = robotSide;
    }

    private void initializeAttributes(OpModeType type, HardwareMap hardwareMap, Gamepad gamepad1, Gamepad gamepad2, Telemetry telemetry) {
        mmSystems.opModeType = type;
        mmSystems.hardwareMap = hardwareMap;
        mmSystems.gamepadEx1 = new GamepadEx(gamepad1);
        mmSystems.gamepadEx2 = new GamepadEx(gamepad2);
        mmSystems.telemetry = telemetry;
        reset(); //reset the scheduler
    }

    private void initializeSystems(OpModeType type) {
        if(type == OpModeType.TELEOP) {
            initTele();
        } else if (type == OpModeType.AUTO) {
            initAuto();
        } else if(type == OpModeType.DEBUG) {
            initDebug();
        }
    }

    private void initTele() {
        //initialize subsystems required for teleop
        initShooterPID();
    }

    private void initAuto() {
        //initialize subsystems required for auto
    }

    private void initDebug() {
        //initialize subsystems required for debug
    }

    //initSubsystems and GamepadKeys
    private void initShooterPID() {
        mmSystems.shooterPID = new ShooterPID();
    }

}
