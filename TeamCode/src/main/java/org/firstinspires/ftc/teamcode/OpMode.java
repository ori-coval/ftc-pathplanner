package org.firstinspires.ftc.teamcode;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "DriveTrein")
public class OpMode extends CommandOpMode{
    DriveTrain driveTrain;



    @Override
    public void initialize() {
        CommandScheduler.getInstance().reset();
        driveTrain = new DriveTrain(hardwareMap.dcMotor.get("motorBL")
                ,hardwareMap.dcMotor.get("motorBR")
                ,hardwareMap.dcMotor.get("motorFR")
                ,hardwareMap.dcMotor.get("motorFL"));
            driveTrain.setDefaultCommand(new TeleopDriveCommand(driveTrain,gamepad1));
    }
}
