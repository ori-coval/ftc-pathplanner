package org.firstinspires.ftc.teamcode.Autonomous;

import com.acmerobotics.dashboard.FtcDashboard;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.missingWpilibClasses.math.kinematics.ChassisSpeeds;
import com.pathplanner.lib.path.PathPlannerPath;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Libraries.MMLib.MMTeleOp;
import org.firstinspires.ftc.teamcode.Utils.OpModeType;

@TeleOp(name = "PPtest")
public class PPTest extends MMTeleOp {
    AutoDrivetrain autoDrivetrain;
    public PPTest() {
        super(OpModeType.NonCompetition.EXPERIMENTING_NO_EXPANSION);
    }

    @Override
    public void onInit() {

        autoDrivetrain = new AutoDrivetrain(hardwareMap.appContext, hardwareMap);

        waitForStart();

//        AutoBuilder.followPath(PathPlannerPath.fromPathFile("test")).schedule();
    autoDrivetrain.drive(0.5);

    }

    @Override
    public void run() {
        super.run();
        autoDrivetrain.odometry.update();
        FtcDashboard.getInstance().getTelemetry().addData("x",autoDrivetrain.odometry.getPosX());
        FtcDashboard.getInstance().getTelemetry().addData("y",autoDrivetrain.odometry.getPosY());
        FtcDashboard.getInstance().getTelemetry().addData("heading",autoDrivetrain.odometry.getHeading());

        FtcDashboard.getInstance().getTelemetry().addData("vel", autoDrivetrain.odometry.getVelX());
        FtcDashboard.getInstance().getTelemetry().update();
    }
}
