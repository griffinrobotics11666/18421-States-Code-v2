package org.firstinspires.ftc.teamcode.util;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;
import org.jetbrains.annotations.NotNull;

import java.util.List;

/**
 * A wrapper for the Tensorflow stuff in the FTC SDK that makes it easier to use.
 */
public class UltimateGoalTfod {
    private static final String TFOD_MODEL_ASSET = "UltimateGoal.tflite";
    private static final String LABEL_FIRST_ELEMENT = "Quad";
    private static final String LABEL_SECOND_ELEMENT = "Single";
    private static final String LABEL_NO_ELEMENT = "None";

    private static Double FIRST_WEIGHT = 1.0;
    private static Double SECOND_WEIGHT = 1.0;
    private static Double NONE_WEIGHT = 0.1;

    private double firstConfidence;
    private double secondConfidence;
    private double noneConfidence;

    private VuforiaLocalizer vuforia;
    private TFObjectDetector tfod;

    private String visibleTarget;
    private boolean targetVisible = false;
    private boolean activated = false;

    public UltimateGoalTfod(VuforiaLocalizer vuforia, @NotNull HardwareMap hardwareMap){
        this.vuforia = vuforia;
        int tfodMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
                "tfodMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
        tfodParameters.minResultConfidence = 0.8f;
        tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);
        tfod.loadModelFromAsset(TFOD_MODEL_ASSET, LABEL_FIRST_ELEMENT, LABEL_SECOND_ELEMENT);
    }
    public UltimateGoalTfod(VuforiaLocalizer vuforia, @NotNull HardwareMap hardwareMap, Double firstWeight, Double secondWeight, Double noneWeight){
        this.vuforia = vuforia;
        int tfodMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
                "tfodMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
        tfodParameters.minResultConfidence = 0.8f;
        tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);
        tfod.loadModelFromAsset(TFOD_MODEL_ASSET, LABEL_FIRST_ELEMENT, LABEL_SECOND_ELEMENT);
        FIRST_WEIGHT = firstWeight;
        SECOND_WEIGHT = secondWeight;
        NONE_WEIGHT = noneWeight;
    }

    public void activate() {
        tfod.activate();
        firstConfidence = 0.0;
        secondConfidence = 0.0;
        noneConfidence = 0.0;
        activated = true;
    }

    public void deactivate(){
        tfod.deactivate();
        activated = false;
    }

    public void shutdown(){
        tfod.shutdown();
    }

    public boolean targetVisible() {
        return targetVisible;
    }

    public String visibleTarget() {
        return visibleTarget;
    }

    public boolean isActivated(){
        return activated;
    }

    public void update(){
        // getUpdatedRecognitions() will return null if no new information is available since
        // the last time that call was made.
        List<Recognition> updatedRecognitions = tfod.getRecognitions();
        targetVisible = false;
        if (updatedRecognitions != null) {
            if(!updatedRecognitions.isEmpty()) {
                targetVisible = true;
                // step through the list of recognitions and display boundary info.
                for (Recognition recognition: updatedRecognitions) {
                    switch(recognition.getLabel()){
                        case LABEL_FIRST_ELEMENT: {
                            firstConfidence += FIRST_WEIGHT;
                            break;
                        }
                        case LABEL_SECOND_ELEMENT: {
                            secondConfidence += SECOND_WEIGHT;
                            break;
                        }
                    }
                }
            }
            else {
                noneConfidence += NONE_WEIGHT;
            }
        }
        else {
            noneConfidence += NONE_WEIGHT;
        }

        if(firstConfidence > noneConfidence && firstConfidence > secondConfidence){
            visibleTarget = LABEL_FIRST_ELEMENT;
        }
        else if(secondConfidence > noneConfidence && secondConfidence > firstConfidence){
            visibleTarget = LABEL_SECOND_ELEMENT;
        }
        else if(noneConfidence > firstConfidence && noneConfidence > secondConfidence){
            visibleTarget = LABEL_NO_ELEMENT;
        }
        else {
            visibleTarget = LABEL_NO_ELEMENT;
        }
    }
}
