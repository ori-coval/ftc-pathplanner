package org.firstinspires.ftc.teamcode.TeleOps;

import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.CuttlefishFTCBridge.src.devices.MMServo;
import org.firstinspires.ftc.teamcode.MMLib.MMPoint2D;
import org.firstinspires.ftc.teamcode.MMLib.MMTeleOp;
import org.firstinspires.ftc.teamcode.MMLib.MMUtils;
import org.firstinspires.ftc.teamcode.MMRobot;

@TeleOp
public class CartridgeTest extends MMTeleOp {
    public CartridgeTest() {
        super(true);
    }

    MMRobot mmRobot = MMRobot.getInstance();

    @Override
    public void main() {
        MMServo servo = mmRobot.mmSystems.expansionHub.getServo(4);

        mmRobot.mmSystems.gamepadEx1.getGamepadButton(GamepadKeys.Button.A).whileActiveOnce(
                new CommandBase() {
                    @Override
                    public void execute() {
                        servo.setPosition(
                                1 - MMUtils.mapValuesLinear(
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
