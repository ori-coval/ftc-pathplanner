package org.firstinspires.ftc.teamcode.TeleOps;

import com.arcrobotics.ftclib.command.button.Trigger;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.Commands.IntakeByPower;
import org.firstinspires.ftc.teamcode.Commands.IntakeByToggle;
import org.firstinspires.ftc.teamcode.Commands.LinearIntakeCommand;
import org.firstinspires.ftc.teamcode.Libraries.MMLib.DriveTrain.Commands.ResetFieldOrientedCommand;
import org.firstinspires.ftc.teamcode.Libraries.MMLib.DriveTrain.Subsystem.MMDriveTrain;
import org.firstinspires.ftc.teamcode.Libraries.MMLib.MMTeleOp;
import org.firstinspires.ftc.teamcode.MMInitMethods;
import org.firstinspires.ftc.teamcode.MMRobot;
import org.firstinspires.ftc.teamcode.MMSystems;
import org.firstinspires.ftc.teamcode.SubSystems.DriveTrain;
import org.firstinspires.ftc.teamcode.Utils.OpModeType;

@TeleOp
public class TeleopDrive extends MMTeleOp {
    public TeleopDrive() {
        super(OpModeType.NonCompetition.EXPERIMENTING_NO_EXPANSION);
    }

    @Override
    public void onInit() {
        MMInitMethods.initDriveTrain();
        MMInitMethods.initIntake();

        Trigger leftTriggerCondition = new Trigger(()-> gamepad1.left_trigger>0.1);
        leftTriggerCondition.toggleWhenActive(new LinearIntakeCommand());

        MMRobot.getInstance().mmSystems.gamepadEx1.getGamepadButton(GamepadKeys.Button.B).whenPressed(new IntakeByToggle());

        MMRobot.getInstance().mmSystems.gamepadEx1.getGamepadButton(GamepadKeys.Button.A).whenPressed(new ResetFieldOrientedCommand());

    }

    @Override
    public void run() {
        super.run();
        telemetry.addData("yaw", MMRobot.getInstance().mmSystems.imu.getYawInDegrees());
        telemetry.update();
    }

}