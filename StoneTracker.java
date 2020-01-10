package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.hardware.camera.Camera;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import java.util.List;
import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer.CameraDirection;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;


public class StoneTracker {
    private static final String TFOD_MODEL_ASSET = "Skystone.tflite";
    private static final String LABEL_FIRST_ELEMENT = "Stone";
    private static final String LABEL_SECOND_ELEMENT = "Skystone";
    private static final String VUFORIA_KEY =
            "AYME2p3/////AAABmeVjQ60ipUjutO8p+A5CWZgB9Kpp3Sm0nTmkXigsOPptZ5kOtQO3KMhDVl+dbGxPtlPm7RCZMPj6Vu1DnCA01y4cz9S6Bh5m5jEecvtvw6c11JFf3jFr63uqQkPEemN8sjJJmFeMgu9PyxAREcPwn86rpRhYrAq7m3RtuT+UjzVOt9fZsp33URsKgsgraY932jDOa033slaKf2sh829y23jyMmPTC1yxU+fxDsDoePByS9AhiJG+c1WWF/w8VS94ORuIXbqc+nBcgGYpLXtFYLZLAPTyNkCgWWtVMDvoFV/SD8v3C+/cpz4+uIjzfqtqimyZlb8OpO/xv/kvXTnHZo0AXGR4tZNvAleetU4M9VIf";

    private VuforiaLocalizer vuforia;
    private TFObjectDetector tfod;
    
    public List<Recognition> updatedRecognitions = null;
    public int stonecount = 0;
    public Recognition stone = null;
    public Recognition skystone = null;
    
    public void init(HardwareMap hardwareMap) {
        // initialize the Vuforia engine
        VuforiaLocalizer.Parameters vparams = new VuforiaLocalizer.Parameters();
        vparams.vuforiaLicenseKey = VUFORIA_KEY;
        vparams.cameraName = hardwareMap.get(WebcamName.class, "webcam");
        vuforia = ClassFactory.getInstance().createVuforia(vparams);
        
        // initialize TensorFlow Object Detector
        int tfodMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
            "tfodMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
        tfodParameters.minimumConfidence = 0.7;
        tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);
        tfod.loadModelFromAsset(TFOD_MODEL_ASSET, LABEL_FIRST_ELEMENT, LABEL_SECOND_ELEMENT);
    }

    public void activate() {
        if (tfod != null) tfod.activate();
    }

    public void stop() {
        if (tfod != null) tfod.shutdown();
    }    

    public int scan() {
        if (tfod == null) return 0;
        updatedRecognitions = tfod.getUpdatedRecognitions();
        if (updatedRecognitions != null) {
            stone = null; 
            skystone = null;
            for (Recognition recognition : updatedRecognitions) {
                stone = recognition;
                if (recognition.getLabel().equals(LABEL_SECOND_ELEMENT)) {
                   skystone = recognition;
                }
            }
            stonecount = updatedRecognitions.size();
            return stonecount;
        }
        return 0;
    }
    
    public double closestSkyStoneHeight() {
        if (skystone != null) {
            return skystone.getHeight();
        }
        return 0;
    }

    public double closestSkyStoneAngle() {
        if (skystone != null) {
            return skystone.estimateAngleToObject(AngleUnit.DEGREES);
        }
        return 0;
    }

    public double closestStoneHeight() {
        if (stone != null) {
            return stone.getHeight();
        }
        return 0;
    }

    public double closestStoneAngle() {
        if (stone != null) {
            return stone.estimateAngleToObject(AngleUnit.DEGREES);
        }
        return 0;
    }

    public String toString() {
        String returnString = "";
        if (updatedRecognitions == null) {
            return "(none)";
        } else {
            for (Recognition recognition : updatedRecognitions) {
                stone = recognition;
                returnString = returnString + String.format("%s @ (%.0f,%.0f) %.0fx%.0f %.2fdeg",
                    stone.getLabel(),
                    stone.getLeft() + stone.getWidth()/2,
                    stone.getTop() + stone.getHeight()/2,
                    stone.getWidth(),
                    stone.getHeight(),
                    stone.estimateAngleToObject(AngleUnit.DEGREES));
            }
            return returnString;
        }
    }
   
}
