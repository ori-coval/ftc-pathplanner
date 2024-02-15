package org.firstinspires.ftc.teamcode.Commands.auto;

import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;

import org.firstinspires.ftc.teamcode.ArmPosition;
import org.firstinspires.ftc.teamcode.Commands.armCommands.multiSystem.ArmGetToPosition;
import org.firstinspires.ftc.teamcode.RoadRunner.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.SubSystems.AntiTurret;
import org.firstinspires.ftc.teamcode.SubSystems.Cartridge;
import org.firstinspires.ftc.teamcode.SubSystems.Elbow;
import org.firstinspires.ftc.teamcode.SubSystems.Elevator;
import org.firstinspires.ftc.teamcode.SubSystems.Extender;
import org.firstinspires.ftc.teamcode.SubSystems.Turret;

public class Parking extends SequentialCommandGroup {
    public Parking(SampleMecanumDrive driveTrain, Elevator elevator, Extender extender, Elbow elbow, Turret turret, AntiTurret antiTurret, Cartridge cartridge){
        super(
            new ParallelCommandGroup(new TrajectoryFollowerCommand(Trajectories.get("1/2 meter backwards from backstage"), driveTrain)),
            new ArmGetToPosition(elevator, elbow, extender, turret, antiTurret, ArmPosition.INTAKE, true),
            new ParallelCommandGroup(new TrajectoryFollowerCommand(Trajectories.get("1/2 meter forwards to backstage"), driveTrain))

        );
    }
}
