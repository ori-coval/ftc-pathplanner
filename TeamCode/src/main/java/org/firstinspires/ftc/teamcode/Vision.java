package org.firstinspires.ftc.teamcode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.tfod.TfodProcessor;

public class Vision {
    private VisionPortal visionPortal;
    private TfodProcessor tfod;
    private WebcamName webcamName;
    public Vision(WebcamName webcamName){
        this.webcamName = webcamName;

        tfod = new TfodProcessor.Builder().build();

        VisionPortal.Builder builder = new VisionPortal.Builder();
        builder.setCamera(webcamName);
        builder.addProcessor(tfod);
        visionPortal = builder.build();
    }

    public int getAmountOfDetections(){
        return tfod.getRecognitions().size();
    }





}
