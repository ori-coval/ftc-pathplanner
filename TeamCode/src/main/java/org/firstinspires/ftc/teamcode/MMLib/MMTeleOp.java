package org.firstinspires.ftc.teamcode.MMLib;

import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.InstantCommand;

import org.firstinspires.ftc.teamcode.MMRobot;
import org.firstinspires.ftc.teamcode.Utils.AllianceColor;
import org.firstinspires.ftc.teamcode.Utils.AllianceSide;
import org.firstinspires.ftc.teamcode.Utils.OpModeType;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;


/**
 * this class represents a wrapper for the default opmode
 * rn this can be used for the competition teleop (i.e. color and sides)
 * if it used while calling the side and/or color val it initializes all the subsystems in the robot
 */
public abstract class MMTeleOp extends CommandOpMode {

    private final MMRobot mmRobot = MMRobot.getInstance();

    private OpModeType opModeType = OpModeType.TELEOP;

    private AllianceColor allianceColor;
    private AllianceSide allianceSide;

    private final List<Runnable> runOnInit = new ArrayList<>();
    private final List<Command> commandsOnRun = new ArrayList<>();


    /**
     * use this if u want to initialize the subsystems and buttons urself
     */
    public MMTeleOp(boolean withExpansion) {
        opModeType = withExpansion ? OpModeType.EXPERIMENTING : OpModeType.EXPERIMENTING_NO_EXPANSION;
    }

    public MMTeleOp(AllianceColor allianceColor) {
        this.allianceColor = allianceColor;
    }

    public MMTeleOp(AllianceColor allianceColor, AllianceSide allianceSide) {
        this(allianceColor);
        this.allianceSide = allianceSide;
    }

    @Override
    public void initialize() {

        robotInit();

        main();

        scheduleCommandsAndRun();

    }

    private void robotInit() {
        mmRobot.init(opModeType, allianceColor, allianceSide, hardwareMap, gamepad1, gamepad2, telemetry);
    }

    /**
     * this method helps you add runnable or commands that will happen on init or run.
     * due to the fact that the commandScheduler run only when opmode is active,
     * you can't run command on init, but you can run runnable on init, and commands right after opmode is active.
     * use addRunnableOnInit and/or addCommandsOnRun.
     * leave it empty if you don't need them.
     * this ofc can also be used to add custom buttons (in experimenting) or whatever you might want.
     */
    public abstract void main();

    public void addRunnableOnInit(Runnable... runOnInit) {
        this.runOnInit.addAll(Arrays.asList(runOnInit));
    }

    public void addCommandsOnRun(Command... commandsOnRun) {
        this.commandsOnRun.add(new InstantCommand().andThen(commandsOnRun));
        /*this was in order to solve the commandScheduler problem
          the problem was that the scheduler for some reason always ran the first instant command even tho it wasnt on yet*/
    }

    private void scheduleCommandsAndRun() {
        for(Runnable runnable : runOnInit) {
            runnable.run();
        }

        for(Command command : commandsOnRun) {
            schedule(command);
        }
    }

}