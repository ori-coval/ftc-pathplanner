package org.firstinspires.ftc.teamcode.MMLib;

import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.InstantCommand;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.CuttlefishFTCBridge.src.devices.MMRevHub;
import org.firstinspires.ftc.teamcode.MMRobot;
import org.firstinspires.ftc.teamcode.Utils.AllianceColor;
import org.firstinspires.ftc.teamcode.Utils.AllianceSide;
import org.firstinspires.ftc.teamcode.Utils.OpModeType;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;


/**
 * this class represents a wrapper for the default opmode.
 * this can be used for the competition teleop (i.e. color and sides)
 * if it is used while calling the side and/or color val it initializes the MMRobot with the {@link OpModeType#TELEOP} type.
 * u might also use the constructor that only requires a boolean, that initializes an {@link OpModeType#EXPERIMENTING} teleop,
 * this mode does not initializes any subsystems, it does give u the control hub and depending on
 * the boolean u specified, the expansion hub ({@link OpModeType#EXPERIMENTING_NO_EXPANSION}).
 * this type of teleop will let u initialize all the hardware u need
 * urself in the {@link #main()} method.
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
     * @param withExpansion whether to activate the expansion or not
     */
    public MMTeleOp(boolean withExpansion) {
        opModeType = withExpansion ? OpModeType.EXPERIMENTING : OpModeType.EXPERIMENTING_NO_EXPANSION;
    }

    /**
     * use this to initialize the normal teleop with the color only.
     * this is what u might want if the sides are irrelevant in ur teleop.
     * @param allianceColor the color of the alliance
     */
    public MMTeleOp(AllianceColor allianceColor) {
        this.allianceColor = allianceColor;
    }

    /**
     * this can be used if ur opmodes are different according to ur alliance side.
     * (u might also use this if ur opmode represents autonomous)
     * @param allianceColor the color of the alliance
     * @param allianceSide the side of the alliance
     */
    public MMTeleOp(AllianceColor allianceColor, AllianceSide allianceSide) {
        this(allianceColor);
        this.allianceSide = allianceSide;
    }

    /**
     * this method runs on init
     * first it initializes the robot instance,
     * then it runs the implementation of the main method the user specified,
     * and at last it schedule the commands and runnables.
     * the order is important in this method,
     * if u run the main method without initializing the robot first u wont be able to do anything really,
     * cause u need the robot instance to get to the subsystems.
     * and the {@link #scheduleCommandsAndRun()} runs the lists added in the main method.
     */
    @Override
    public void initialize() {

        robotInit();

        main();

        scheduleCommandsAndRun();

    }

    /**
     * this initializes the robot instance.
     */
    private void robotInit() {
        mmRobot.init(opModeType, allianceColor, allianceSide, hardwareMap, gamepad1, gamepad2, telemetry);
    }

    /**
     * this method helps you add {@link Runnable} or {@link Command} that will happen on init or run.
     * due to the fact that the {@link com.arcrobotics.ftclib.command.CommandScheduler CommandScheduler} run only when opmode is active,
     * you can't run command on init, but you can run runnable on init, and commands right after opmode is active.
     * use {@link #addRunnableOnInit(Runnable...)} and/or {@link #addCommandsOnRun(Command...)}.
     * leave it empty if you don't need them.
     * this ofc can also be used to add custom buttons (in experimenting) or whatever you might want.
     */
    public abstract void main();

    /**
     * this method helps u add methods that will run on the init stage. (before the actual match)
     * @param runOnInit methods to run
     */
    public void addRunnableOnInit(Runnable... runOnInit) {
        this.runOnInit.addAll(Arrays.asList(runOnInit));
    }

    /**
     * this method lets u add command to run after the init stage. (when the match begins)
     * @param commandsOnRun commands to schedule
     */
    public void addCommandsOnRun(Command... commandsOnRun) {
        this.commandsOnRun.add(new InstantCommand().andThen(commandsOnRun));
        /*this was in order to solve the commandScheduler problem
          the problem was that the scheduler for some reason always ran the first instant command even tho it wasnt on yet*/
    }

    /**
     * this method runs the actions and schedules the commands that were specified in the main method.
     * commands and methods that were added with the {@link #addRunnableOnInit(Runnable...)} and {@link  #addCommandsOnRun(Command...)}
     */
    private void scheduleCommandsAndRun() {
        for(Runnable runnable : runOnInit) {
            runnable.run();
        }

        for(Command command : commandsOnRun) {
            schedule(command);
        }
    }

    /**
     * the {@link MMRevHub#pullBulkData()} method needs to be called in order to update the (non-i2c) sensors.
     * the {@link Telemetry#update()} method is used to update the telemetry on every iteration of ur opmode.
     * <p>
     * this is ur loop in the opmode.
     */
    @Override
    public void run() {
        super.run();
        mmRobot.mmSystems.controlHub.pullBulkData();
        mmRobot.mmSystems.expansionHub.pullBulkData();
        telemetry.update();
    }
}
