package org.firstinspires.ftc.teamcode.Commands.multiSystem;

import com.arcrobotics.ftclib.command.ConditionalCommand;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;

import org.firstinspires.ftc.teamcode.ArmPosition;
import org.firstinspires.ftc.teamcode.ArmPositionSelector;
import org.firstinspires.ftc.teamcode.SubSystems.AntiTurret;
import org.firstinspires.ftc.teamcode.SubSystems.Elbow;
import org.firstinspires.ftc.teamcode.SubSystems.Elevator;
import org.firstinspires.ftc.teamcode.SubSystems.Extender;
import org.firstinspires.ftc.teamcode.SubSystems.Turret;

public class ArmGetToSelectedPositionTest extends SequentialCommandGroup {
    private static final ArmPosition[] armPositions = ArmPosition.values();
    public ArmGetToSelectedPositionTest(Elevator elevator, Elbow elbow, Extender extender, Turret turret, AntiTurret antiTurret) {
        for (ArmPosition armPosition : armPositions) {
            addCommands(new ConditionalCommand(
                    new ArmGetToPosition(elevator, elbow, extender, turret, antiTurret, armPosition, ArmPositionSelector.getIsLeftOfBoard()),
                    new InstantCommand(),
                    () -> armPosition == ArmPositionSelector.getPosition()
            ));
        }
    }
}