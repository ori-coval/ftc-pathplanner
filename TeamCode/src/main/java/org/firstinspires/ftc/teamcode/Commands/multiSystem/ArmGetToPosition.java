package org.firstinspires.ftc.teamcode.Commands.multiSystem;

import com.arcrobotics.ftclib.command.ConditionalCommand;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;

import org.firstinspires.ftc.teamcode.ArmPosition;
import org.firstinspires.ftc.teamcode.SubSystems.AntiTurret;
import org.firstinspires.ftc.teamcode.SubSystems.Elbow;
import org.firstinspires.ftc.teamcode.SubSystems.Elevator;
import org.firstinspires.ftc.teamcode.SubSystems.Extender;
import org.firstinspires.ftc.teamcode.SubSystems.Turret;

public class ArmGetToPosition extends ParallelCommandGroup {
    public static ArmPosition lastPosition = ArmPosition.INTAKE;
    private ArmPosition targetPosition;
    public ArmGetToPosition(Elevator elevator, Elbow elbow, Extender extender, Turret turret, AntiTurret antiTurret, ArmPosition position, boolean isLeftOfBoard) {
        addCommands(
                new ConditionalCommand(
                        new UnsafeMoveArm(elevator, elbow, extender, turret, antiTurret, position, isLeftOfBoard),
                        new UnsafeMoveArm(elevator, elbow, extender, turret, antiTurret, ArmPosition.SAFE_PLACE, isLeftOfBoard).andThen(
                                new UnsafeMoveArm(elevator, elbow, extender, turret, antiTurret, position, isLeftOfBoard)
                        ),
                        ()-> lastPosition.getCluster() == position.getCluster()
                )
        );
        targetPosition = position; //
    }


    @Override
    public void initialize() {
        super.initialize();
        lastPosition = targetPosition;
    }
}
