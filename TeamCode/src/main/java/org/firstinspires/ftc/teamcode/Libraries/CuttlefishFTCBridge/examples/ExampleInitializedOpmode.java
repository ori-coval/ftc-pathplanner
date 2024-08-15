package org.firstinspires.ftc.teamcode.Libraries.CuttlefishFTCBridge.examples;

import com.roboctopi.cuttlefish.controller.MecanumController;
import com.roboctopi.cuttlefish.controller.PTPController;
import com.roboctopi.cuttlefish.localizer.ThreeEncoderLocalizer;
import com.roboctopi.cuttlefish.queue.TaskQueue;
import com.roboctopi.cuttlefish.utils.Direction;

import org.firstinspires.ftc.teamcode.Libraries.CuttlefishFTCBridge.src.devices.MMEncoder;
import org.firstinspires.ftc.teamcode.Libraries.CuttlefishFTCBridge.src.devices.MMMotor;
import org.firstinspires.ftc.teamcode.Libraries.CuttlefishFTCBridge.src.devices.MMRevHub;
import org.firstinspires.ftc.teamcode.Libraries.CuttlefishFTCBridge.src.opmodeTypes.GamepadOpMode;


public abstract class ExampleInitializedOpmode extends GamepadOpMode {
    // Declare the rev hubs. If you only have one hub connected you can delete one of these
    public MMRevHub ctrlHub;
    public MMRevHub expHub;


    // Declare the chassis motors
    public MMMotor leftFrontMotor ;
    public MMMotor rightFrontMotor;
    public MMMotor rightBackMotor ;
    public MMMotor leftBackMotor  ;

    // Declare the mecanum controller
    public MecanumController chassis;

    // Declare the localizer
    public ThreeEncoderLocalizer encoderLocalizer;

    // Declare the PTPController
    public PTPController ptpController;

    // Declare the task queue
    public TaskQueue queue;


    @Override
    public void onInit()
    {
        /*
        Define the rev hubs
        If this throws an error, try getting the hubs by name
        You can find the name of the hubs in the config file
        */
        ctrlHub = new MMRevHub(hardwareMap, MMRevHub.HubTypes.CONTROL_HUB);
        expHub = new MMRevHub(hardwareMap,"Expansion Hub 1");

        /*
        Get the chassis motors
        Make sure to replace the ports and hubs of each motor with the corresponding ports and hubs on your robot
         */
        leftFrontMotor  = ctrlHub.getMotor(1);
        rightFrontMotor = ctrlHub.getMotor(0);
        rightBackMotor  = ctrlHub.getMotor(2);
        leftBackMotor   = ctrlHub.getMotor(3);

        /*
        Reverse the motors on the left
        If your robot doesn't go in the correct direction (e.g. it turns when you tell it to go forward) ,
        you probably need to change the motors that are reversed
        */
        leftBackMotor .setDirection(Direction.REVERSE);
        leftFrontMotor.setDirection(Direction.REVERSE);

        //Initialize and set the direction of the encoders
        MMEncoder leftEncoder  = expHub .getEncoder(0,720*4);
        MMEncoder sideEncoder  = expHub .getEncoder(3,720*4);
        MMEncoder rightEncoder = ctrlHub.getEncoder(3,720*4);
        rightEncoder.setDirection(Direction.REVERSE);
        sideEncoder.setDirection(Direction.REVERSE);

        // Initialize the mecanum controller
        chassis = new MecanumController(rightFrontMotor,rightBackMotor,leftFrontMotor,leftBackMotor);

        // Initialize the localizer
        encoderLocalizer = new ThreeEncoderLocalizer(
                leftEncoder  , // Left
                sideEncoder  , // Side
                rightEncoder , // Right
                29,
                234.0,
                1.0
        );

        // Initialize the PTP Controller
        ptpController = new PTPController(chassis, encoderLocalizer);

        // Initialize the queue
        queue = new TaskQueue();

    }
    @Override
    public void main() {
    }
    public void mainLoop()
    {
        // Pull bulk data from both hubs
        ctrlHub.pullBulkData();
        expHub.pullBulkData();

        // Update the localizer
        encoderLocalizer.update();

        // Update the queue
        queue.update();
    }
}