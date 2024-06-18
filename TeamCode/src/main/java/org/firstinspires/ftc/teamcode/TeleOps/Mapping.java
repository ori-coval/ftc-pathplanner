package org.firstinspires.ftc.teamcode.TeleOps;

import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.CuttlefishFTCBridge.src.devices.CuttleServo;
import org.firstinspires.ftc.teamcode.MMLib.MMPoint2D;
import org.firstinspires.ftc.teamcode.MMLib.MMTeleOp;
import org.firstinspires.ftc.teamcode.MMLib.MMUtils;
import org.firstinspires.ftc.teamcode.MMRobot;

@TeleOp
public class Mapping extends MMTeleOp {

    public Mapping() {
        super(false);
    }

    MMRobot mmRobot = MMRobot.getInstance();

    CuttleServo servo;

    @Override
    public void main() {

        servo = mmRobot.mmSystems.controlHub.getServo(0);

        mmRobot.mmSystems.gamepadEx1.getGamepadButton(GamepadKeys.Button.A).whileActiveOnce(
                new CommandBase() {
                    @Override
                    public void execute() {
                        servo.setPosition(
                                MMUtils.mapValuesLinear(
                                        mmRobot.mmSystems.gamepadEx1.getLeftX(),
                                        new MMPoint2D(-1, 0),
                                        new MMPoint2D(1, 1)
                                )
                        );
                    }
                }
        );

    }
}
