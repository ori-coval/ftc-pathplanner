package org.firstinspires.ftc.teamcode.Commands.auto;


import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.Commands.armCommands.multiSystem.BackToIntake;
import org.firstinspires.ftc.teamcode.Commands.armCommands.turret.RotateTurretByPID;
import org.firstinspires.ftc.teamcode.Commands.intakeLifter.IntakeCollectFromStack;
import org.firstinspires.ftc.teamcode.Commands.intakeLifter.IntakeSetStackPosition;
import org.firstinspires.ftc.teamcode.Commands.intakeRoller.IntakeRotate;
import org.firstinspires.ftc.teamcode.Commands.utils.SideCommandSwitch;
import org.firstinspires.ftc.teamcode.RoadRunner.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.SubSystems.AntiTurret;
import org.firstinspires.ftc.teamcode.SubSystems.Cartridge;
import org.firstinspires.ftc.teamcode.SubSystems.Elbow;
import org.firstinspires.ftc.teamcode.SubSystems.Elevator;
import org.firstinspires.ftc.teamcode.SubSystems.Extender;
import org.firstinspires.ftc.teamcode.SubSystems.Turret;
import org.firstinspires.ftc.teamcode.Utils.Side;
import org.firstinspires.ftc.teamcode.SubSystems.Intake;

import java.util.Collection;

public class GoFromSpikeMarkToStack extends SequentialCommandGroup {
    public GoFromSpikeMarkToStack(SampleMecanumDrive driveTrain, Intake intake, Side side, Elevator elevator, Extender extender, Elbow elbow, Turret turret, AntiTurret antiTurret, Cartridge cartridge) {
        super(
                new IntakeSetStackPosition(intake.lifter, Intake.LifterPosition.FIRST_PIXEL),
                new InstantCommand(() -> intake.roller.setPower(intake.roller.COLLECT_POWER)),
                new SideCommandSwitch(
                        new TrajectoryFollowerCommand(Trajectories.get("Driving to stack while avoiding pixel on Left"), driveTrain),
                        new TrajectoryFollowerCommand(Trajectories.get("Driving to stack while avoiding pixel on Center"), driveTrain),
                        new TrajectoryFollowerCommand(Trajectories.get("Driving to stack while avoiding pixel on Right"), driveTrain),
                        () -> side
                )



                /*,

                new ParallelCommandGroup(
                    new SideCommandSwitch(
                            new TrajectoryFollowerCommand(Trajectories.get("Driving to stack while avoiding pixel on Left"), driveTrain),
                            new TrajectoryFollowerCommand(Trajectories.get("Driving to stack while avoiding pixel on Center"), driveTrain),
                            new TrajectoryFollowerCommand(Trajectories.get("Driving to stack while avoiding pixel on Right"), driveTrain),
                            () -> side
                    ),
                    new WaitCommand(400).andThen(new BackToIntake(elevator, elbow, extender, turret, antiTurret, cartridge))
                )
                */

        );
    }
}
