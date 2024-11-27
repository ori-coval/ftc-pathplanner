package org.firstinspires.ftc.teamcode.Autonomous;

import android.content.Context;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.MecanumKinematics;
import com.arcrobotics.ftclib.command.Subsystem;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.controller.PIDController;
import com.arcrobotics.ftclib.controller.PIDFController;
import com.arcrobotics.ftclib.controller.wpilibcontroller.SimpleMotorFeedforward;
import com.arcrobotics.ftclib.drivebase.MecanumDrive;
import com.arcrobotics.ftclib.geometry.Vector2d;
import com.arcrobotics.ftclib.kinematics.wpilibkinematics.MecanumDriveWheelSpeeds;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.missingWpilibClasses.kinematics.MecanumDriveKinematics;
import com.pathplanner.lib.missingWpilibClasses.math.geometry.Translation2d;
import com.pathplanner.lib.missingWpilibClasses.math.kinematics.ChassisSpeeds;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.roboctopi.cuttlefish.controller.MecanumController;
import com.roboctopi.cuttlefish.utils.Direction;

import org.apache.commons.math3.linear.MatrixUtils;
import org.apache.commons.math3.linear.RealMatrix;
import org.apache.commons.math3.linear.RealVector;
import org.firstinspires.ftc.teamcode.Libraries.CuttlefishFTCBridge.src.devices.CuttleMotor;
import org.firstinspires.ftc.teamcode.MMRobot;
import org.firstinspires.ftc.teamcode.Utils.Configuration;

@Config
public class AutoDrivetrain extends SubsystemBase {
    MMRobot mmRobot = MMRobot.getInstance();

    GoBildaPinpointDriver odometry;

    private final CuttleMotor motorFR;
    private final CuttleMotor motorFL;
    private final CuttleMotor motorBL;
    private final CuttleMotor motorBR;
    private final SimpleMotorFeedforward feedforward = new SimpleMotorFeedforward(0, 1);

    MecanumDriveKinematics kinematics = new MecanumDriveKinematics(
            new Translation2d(),
            new Translation2d(),
            new Translation2d(),
            new Translation2d());



    public AutoDrivetrain(Context context, HardwareMap hardwareMap) {
        super(); //register this subsystem, in order to schedule default command later on.

        odometry = hardwareMap.get(GoBildaPinpointDriver.class, "odometry");
        odometry.setOffsets(0, -10);
        odometry.setEncoderResolution(13.26291192f);// * 1000);
        odometry.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.FORWARD, GoBildaPinpointDriver.EncoderDirection.FORWARD);
        odometry.resetPosAndIMU();
        //TODO add recalibrateIMU function to onInit

        motorFL = new CuttleMotor(mmRobot.mmSystems.controlHub, Configuration.DRIVE_TRAIN_FRONT_LEFT);
        motorBL = new CuttleMotor(mmRobot.mmSystems.controlHub, Configuration.DRIVE_TRAIN_BACK_LEFT);
        motorFR = new CuttleMotor(mmRobot.mmSystems.controlHub, Configuration.DRIVE_TRAIN_FRONT_RIGHT);
        motorBR = new CuttleMotor(mmRobot.mmSystems.controlHub, Configuration.DRIVE_TRAIN_BACK_RIGHT);

        motorBR.setDirection(Direction.REVERSE);
        motorFR.setDirection(Direction.REVERSE);

        AutoBuilder.setContext(context);

        AutoBuilder.configureHolonomic(
                odometry::getPosition, // Robot pose supplier
                odometry::setPosition, // Method to reset odometry (will be called if your auto has a starting pose)
                odometry::getVelocity, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
                this::drive, // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds
                new HolonomicPathFollowerConfig( // HolonomicPathFollowerConfig, this should likely live in your Constants class
                        new PIDConstants(1.0, 0.0, 0.0), // Translation PID constants
                        new PIDConstants(1.0, 0.0, 0.0), // Rotation PID constants
                        0.5, // Max module speed, in m/s
                        0.1, // Drive base radius in meters. Distance from robot center to furthest module.
                        new ReplanningConfig() // Default path replanning config. See the API for the options here
                ),
                ()->false,
                this // Reference to this subsystem to set requirements
        );

    }



    public  void  drive(ChassisSpeeds chassisSpeeds) {
        MecanumDriveWheelSpeeds speeds = kinematics.toWheelSpeeds(chassisSpeeds);

        motorFL.setPower(feedforward.calculate(speeds.frontLeftMetersPerSecond));
        motorBL.setPower(feedforward.calculate(speeds.rearLeftMetersPerSecond));
        motorFR.setPower(feedforward.calculate(speeds.frontRightMetersPerSecond));
        motorBR.setPower(feedforward.calculate(speeds.rearRightMetersPerSecond));
    }
    public  void  drive(double speed) {

        motorFL.setPower(speed);
        motorBL.setPower(speed);
        motorFR.setPower(speed);
        motorBR.setPower(speed);
    }


    @Override
    public void periodic() {
        super.periodic();

        updateTelemetry();
    }

    public void updateTelemetry() {
        FtcDashboard.getInstance().getTelemetry().addData("frontLeft", motorFL.getPower());
        FtcDashboard.getInstance().getTelemetry().addData("backLeft", motorBL.getPower());
        FtcDashboard.getInstance().getTelemetry().addData("frontRight", motorFR.getPower());
        FtcDashboard.getInstance().getTelemetry().addData("backRight", motorBR.getPower());


        FtcDashboard.getInstance().getTelemetry().update();
    }

}
