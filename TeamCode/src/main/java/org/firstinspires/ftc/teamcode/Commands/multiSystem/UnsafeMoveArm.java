package org.firstinspires.ftc.teamcode.Commands.multiSystem;

import com.acmerobotics.dashboard.FtcDashboard;
import com.arcrobotics.ftclib.command.ConditionalCommand;

import org.firstinspires.ftc.teamcode.ArmPosition;
import org.firstinspires.ftc.teamcode.SubSystems.AntiTurret;
import org.firstinspires.ftc.teamcode.SubSystems.Elbow;
import org.firstinspires.ftc.teamcode.SubSystems.Elevator;
import org.firstinspires.ftc.teamcode.SubSystems.Extender;
import org.firstinspires.ftc.teamcode.SubSystems.Turret;

public class UnsafeMoveArm extends ConditionalCommand {

    public static ArmPosition lastPosition = ArmPosition.INTAKE;
    public static final long EXTENDER_WAIT_TIME = 250;
    public static final long ELEVATOR_WAIT_TIME = 250;
    private final ArmPosition targetPosition;

    public UnsafeMoveArm(Elevator elevator, Elbow elbow, Extender extender, Turret turret, AntiTurret antiTurret, ArmPosition position, boolean isLeftOfBoard) {
        super(
                new UnsafeMoveArmUp(elevator, elbow, extender, turret, antiTurret, position, isLeftOfBoard),
                new UnsafeMoveArmDown(elevator, elbow, extender, turret, antiTurret, position, isLeftOfBoard),
                () -> (lastPosition.getElevatorHeight() < position.getElevatorHeight())
        );
        targetPosition = position;
    }

    @Override
    public void initialize() {
        super.initialize();
        lastPosition = targetPosition;
    }

    @Override
    public void execute() {
        super.execute();
        FtcDashboard.getInstance().getTelemetry().update();
    }
}
