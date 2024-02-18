package org.firstinspires.ftc.teamcode.Commands.drone;

import com.arcrobotics.ftclib.command.InstantCommand;

import org.firstinspires.ftc.teamcode.SubSystems.DroneLauncher;

public class DroneLauncherSetState extends InstantCommand {
    public DroneLauncherSetState(DroneLauncher droneLauncher, DroneLauncher.State state){
        super(()-> droneLauncher.setState(state) , droneLauncher);
    }

}
