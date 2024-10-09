package org.firstinspires.ftc.teamcode.TeleOps;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;


@TeleOp(name = "Sensor: Limelight3A", group = "Sensor")
public class Teleop extends CommandOpMode {

    private Limelight3A limelight;

    @Override
    public void initialize() {
        limelight = hardwareMap.get(Limelight3A.class, "limelight");

        telemetry.setMsTransmissionInterval(11);

        limelight.start();
    }

    @Override
    public void run() {
        super.run();

        //what color we need to search for
        String searchFor = "blue";
        //if there is a yellow that is a bit farther than blue\red do we want to prioritise yellow
        boolean prioritiseYellow = false;
        //the offset that we will prioritise yellow over blue\red
        int yellowPriorityOffset = 2; //--find a good angle--

        //index 0 is for blue\red cords and index 1 is for yellow
        //this is the structure {{tx,ty,ta},{tx,ty,ta}}
        //it array is a 2D array because we want to compere the result of the blue\red and yellow
        while (opModeIsActive())
        {
            double[][] cords = new double[2][2];

            cords[0] = getResult((searchFor == "blue") ? 1 : 2); //defined pipeline for blue(1) and red(2)
            cords[1] = getResult(0); //defined pipeline for yellow(0)


            //if prioritiseYellow is positive subtract yellowPriorityOffset from cords of yellow
            if (prioritiseYellow) {
                cords[1][0] -= yellowPriorityOffset;
                cords[1][1] -= yellowPriorityOffset;
            }

            //check if we want to aim for blue\red of yellow
            if (prioritiseYellow && cords[1][0] < cords[0][0] && cords[1][1] < cords[0][1]){
                telemetry.addData("tx yellow: ", cords[0][0]);
                telemetry.addData("ty yellow: ", cords[0][1]);
            }
            else {
                telemetry.addData("tx" + searchFor + ": ", cords[0][0]);
                telemetry.addData("ty" + searchFor + ": ", cords[1][1]);
            }
            telemetry.update();
        }
    }

    private double[] getResult(int index) //--make thingy return the array of angles with this structure {tx,ty}--
    {
        limelight.pipelineSwitch(index);
        LLResult result = limelight.getLatestResult();
        if (result != null) {
            if (result.isValid()) {
                return new double[] {result.getTx(), result.getTy()};
            }
        }

        return new double[] {1000, 1000};
    }
}
/*
*                 telemetry.addData("tx" + index + ": ", result.getTx());
                telemetry.addData("ty" + index + ": ", result.getTy());
                telemetry.addData("pipline" + index + ": ", botpose.toString());
*/
