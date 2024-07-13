package org.firstinspires.ftc.teamcode.MMLib.OpModes;

import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.Subsystem;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;

import org.firstinspires.ftc.teamcode.MMLib.MMTeleOp;
import org.firstinspires.ftc.teamcode.MMLib.Utils.MMDeferredCommand;
import org.firstinspires.ftc.teamcode.MMRobot;

import java.util.Arrays;
import java.util.HashSet;
import java.util.stream.Collectors;

public class DeferredCommandTest extends MMTeleOp {

    public DeferredCommandTest() {
        super(false);
    }

    @Override
    public void main() {

        MMRobot.getInstance().mmSystems.gamepadEx1.getGamepadButton(GamepadKeys.Button.X).whenPressed(
                new MMDeferredCommand(
                        InstantCommand::new,
                        new HashSet<Subsystem>() {{
                            add(MMRobot.getInstance().mmSystems.shooter);
                            add(MMRobot.getInstance().mmSystems.shooterIntake);
                        }}
                )
        );


        MMRobot.getInstance().mmSystems.gamepadEx1.getGamepadButton(GamepadKeys.Button.A).whenPressed(
                new MMDeferredCommand(
                        InstantCommand::new,
                        Arrays.stream(
                                new Subsystem[]{
                                        MMRobot.getInstance().mmSystems.shooter,
                                        MMRobot.getInstance().mmSystems.shooterIntake
                                }
                        ).collect(Collectors.toSet())
                )
        );

    }
}
