package org.firstinspires.ftc.teamcode.Commands.multiSystem;

import com.arcrobotics.ftclib.command.ConditionalCommand;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;

import org.firstinspires.ftc.teamcode.ArmPosition;
import org.firstinspires.ftc.teamcode.SubSystems.Elbow;
import org.firstinspires.ftc.teamcode.SubSystems.Elevator;
import org.firstinspires.ftc.teamcode.SubSystems.Extender;
import org.firstinspires.ftc.teamcode.SubSystems.Turret;

public class ArmGetToPosition extends ParallelCommandGroup {
    private Elevator elevator;
    private Extender extender;
    private Elbow elbow;
    private Turret turret;
    private static ArmPosition lastPosition = ArmPosition.INTAKE;

    public ArmGetToPosition(Elevator elevator, Elbow elbow, Extender extender, Turret turret, ArmPosition position, boolean isLeftOfBoard) {
        lastPosition = position;
        this.elevator = elevator;
        this.extender = extender;
        this.elbow = elbow;
        this.turret = turret;
        addCommands(
                new ConditionalCommand(
                        new UnsafeMoveArmParallel(elevator, elbow, extender, turret, position, isLeftOfBoard),
                        new UnsafeMoveArmParallel(elevator, elbow, extender, turret, ArmPosition.SAFE_PLACE, isLeftOfBoard).andThen(
                                new UnsafeMoveArmParallel(elevator, elbow, extender, turret, position, isLeftOfBoard)
                        ),
                        ()-> lastPosition.getCluster()==position.getCluster()
                )
        );
    }
}
