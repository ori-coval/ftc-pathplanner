package org.firstinspires.ftc.teamcode.MMLib.Examples.ElevatorExample;

import com.arcrobotics.ftclib.gamepad.GamepadKeys;

import org.firstinspires.ftc.teamcode.MMInitMethods;
import org.firstinspires.ftc.teamcode.MMLib.MMTeleOp;
import org.firstinspires.ftc.teamcode.MMLib.PID.MMPIDCommand;
import org.firstinspires.ftc.teamcode.MMLib.PID.MMTuningFFCommand;
import org.firstinspires.ftc.teamcode.MMRobot;

public class TeleOpUsageExample extends MMTeleOp {


    public TeleOpUsageExample() {
        super(true);
    }

    @Override
    public void main() {

        MMInitMethods.initExampleElevator();

        MMRobot.getInstance().mmSystems.gamepadEx1.getGamepadButton(GamepadKeys.Button.A).whenPressed(
                new MMPIDCommand(MMRobot.getInstance().mmSystems.exampleElevator, 15)
        );

        MMRobot.getInstance().mmSystems.gamepadEx1.getGamepadButton(GamepadKeys.Button.B).whenPressed(
                new MMPIDCommand(MMRobot.getInstance().mmSystems.exampleElevator, 0)
        );

        MMRobot.getInstance().mmSystems.gamepadEx1.getGamepadButton(GamepadKeys.Button.X).whenPressed(
                new MMTuningFFCommand(MMRobot.getInstance().mmSystems.exampleElevator, 1)
        );

    }
}
