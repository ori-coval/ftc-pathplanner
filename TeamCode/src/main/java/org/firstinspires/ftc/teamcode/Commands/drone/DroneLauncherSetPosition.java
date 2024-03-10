package org.firstinspires.ftc.teamcode.Commands.drone;

import com.arcrobotics.ftclib.command.InstantCommand;

import org.firstinspires.ftc.teamcode.SubSystems.DroneLauncher;

public class DroneLauncherSetPosition extends InstantCommand {
    public DroneLauncherSetPosition(DroneLauncher droneLauncher, double position){
        super(() -> droneLauncher.setPosition(position) , droneLauncher);
    }

}
