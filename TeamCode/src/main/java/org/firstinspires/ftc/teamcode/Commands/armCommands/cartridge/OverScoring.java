package org.firstinspires.ftc.teamcode.Commands.armCommands.cartridge;

import org.firstinspires.ftc.teamcode.RobotControl;
import org.firstinspires.ftc.teamcode.SubSystems.Cartridge;

import java.util.function.BooleanSupplier;

public class OverScoring extends ScoringTeleCommand {
    public OverScoring(RobotControl robot, BooleanSupplier triggerCondition) {
        super(
                robot,
                triggerCondition,
                Cartridge.State.AUTONOMOUS_OPEN
        );
    }
}
