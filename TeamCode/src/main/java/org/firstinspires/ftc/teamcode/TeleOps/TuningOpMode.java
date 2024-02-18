package org.firstinspires.ftc.teamcode.TeleOps;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Commands.armCommands.extender.ExtenderSetPosition;
import org.firstinspires.ftc.teamcode.Commands.utilCommands.ServoTuningCommand;
import org.firstinspires.ftc.teamcode.RobotControl;
import org.firstinspires.ftc.teamcode.SubSystems.Elevator;
import org.firstinspires.ftc.teamcode.SubSystems.Turret;
import org.firstinspires.ftc.teamcode.Utils.Configuration;
import org.firstinspires.ftc.teamcode.SubSystems.AntiTurret;
import org.firstinspires.ftc.teamcode.SubSystems.Cartridge;
import org.firstinspires.ftc.teamcode.SubSystems.DroneLauncher;
import org.firstinspires.ftc.teamcode.SubSystems.Elbow;
import org.firstinspires.ftc.teamcode.SubSystems.Extender;


@TeleOp(name = "TuningOpMode")
public class TuningOpMode extends CommandOpMode {
    RobotControl robot;

    @Override
    public void initialize() {
        CommandScheduler.getInstance().reset();
        robot = new RobotControl(RobotControl.OpModeType.DEBUG, hardwareMap, gamepad1 ,gamepad2, telemetry);
    }


    @Override
    public void run() {
        super.run();

        ServoTuningCommand.telemetry(telemetry);

        telemetry.addLine("----------------------------");

        telemetry.addData("antiTurret pos", robot.antiTurret.getPosition());
        telemetry.addData("elbow", robot.elbow.getServoPosition());
        telemetry.addData("elevator height", robot.elevator.getHeight());
        telemetry.addData("turret angle", robot.turret.getAngle());
        telemetry.addData("Extender pos", robot.extender.getPosition());

        telemetry.update();
    }
}