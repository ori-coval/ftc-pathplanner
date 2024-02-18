package org.firstinspires.ftc.teamcode.Commands.armCommands.multiSystem;

import com.arcrobotics.ftclib.command.ConditionalCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.ArmPosition;
import org.firstinspires.ftc.teamcode.Commands.armCommands.antiTurret.AntiTurretGetToPosition;
import org.firstinspires.ftc.teamcode.Commands.armCommands.cartridge.ScoringFirstPixel;
import org.firstinspires.ftc.teamcode.Commands.armCommands.elbow.ElbowGetToPosition;
import org.firstinspires.ftc.teamcode.Commands.armCommands.elevator.ElevatorGetToHeightPID;
import org.firstinspires.ftc.teamcode.Commands.armCommands.extender.ExtenderSetPosition;
import org.firstinspires.ftc.teamcode.Commands.armCommands.turret.RotateTurretByPID;
import org.firstinspires.ftc.teamcode.RobotControl;
import org.firstinspires.ftc.teamcode.SubSystems.AntiTurret;
import org.firstinspires.ftc.teamcode.SubSystems.Elbow;
import org.firstinspires.ftc.teamcode.SubSystems.Elevator;
import org.firstinspires.ftc.teamcode.SubSystems.Extender;
import org.firstinspires.ftc.teamcode.SubSystems.Turret;

public class UnsafeMoveArm extends ConditionalCommand {
    public static final long EXTENDER_WAIT_TIME = 250;
    public static final long ELEVATOR_WAIT_TIME = 250;

    public UnsafeMoveArm(RobotControl robot, ArmPosition position, boolean isLeftOfBoard) {
        super(
                new UnsafeMoveArmUp(robot, position, isLeftOfBoard),
                new UnsafeMoveArmDown(robot, position, isLeftOfBoard),
                () -> (ArmGetToPosition.lastPosition.getElevatorHeight() < position.getElevatorHeight())
        );
    }
}




/* ConditionalCommand

                 new SequentialCommandGroup(
                        new ElevatorGetToHeightPID(elevator, position.getElevatorHeight()),
                        new RotateTurretByPID(turret, position.getTurretAngle(isLeftOfBoard)),
                        new ElbowGetToPosition(elbow, position.getElbowPosition()), //These are instant commands so their isFinished always true
                        new WaitCommand(EXTENDER_WAIT_TIME), //Trying to avoid elbow's servos overload
                        new ExtenderSetPosition(extender, position.getExtenderPosition()),
                        new AntiTurretGetToPosition(antiTurret, position.getAntiTurretPosition())
                ),
                new SequentialCommandGroup(
                        new RotateTurretByPID(turret, position.getTurretAngle(isLeftOfBoard)),
                        new WaitCommand(ELEVATOR_WAIT_TIME), //avoiding elevator's shaking while going down too fast.
                        new ElevatorGetToHeightPID(elevator, position.getElevatorHeight()),
                        new AntiTurretGetToPosition(antiTurret, position.getAntiTurretPosition()),
                        new ConditionalCommand(
                                new SequentialCommandGroup(
                                new ElbowGetToPosition(elbow, position.getElbowPosition()),
                                new WaitCommand(EXTENDER_WAIT_TIME),
                                new ExtenderSetPosition(extender, position.getExtenderPosition())
                                ),
                                new SequentialCommandGroup(
                                    new ExtenderSetPosition(extender, position.getExtenderPosition()),
                                    new WaitCommand(EXTENDER_WAIT_TIME),
                                    new ElbowGetToPosition(elbow, position.getElbowPosition())
                                ),
                                () -> (lastPosition.getExtenderPosition().getServoPositionAsDouble() < position.getExtenderPosition().getServoPositionAsDouble())
                        ),
                new ExtenderSetPosition(extender, position.getExtenderPosition()),
                new ElbowGetToPosition(elbow, position.getElbowPosition())
                ),
                () -> (lastPosition.getElevatorHeight() < position.getElevatorHeight())
 */