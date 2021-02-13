package org.firstinspires.ftc.teamcode.util;

import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;

/**
 * A wrapper for the Vuforia stuff in the FTC SDK that makes it easier to use.
 */
public class VuforiaUtil {

//    private float phoneXRotate    = 0;
//    private float phoneYRotate    = 0;
//    private float phoneZRotate    = 0;

    public static class CameraDirection {
        public static final float FORWARD = 90;
        public static final float BACK = -90;
        public static final float PORTRAIT = 90;
        public static final float LANDSCAPE = 0;
    }

    public static class CameraState {
        public float XRotate;
        public float YRotate;
        public float ZRotate;
        public float ForwardDisplacement;
        public float LeftDisplacement;
        public float VerticalDisplacement;
        public VuforiaLocalizer.CameraDirection cameraDirection;
        public CameraState(VuforiaLocalizer.CameraDirection cameraDirection, float CameraOrientation, float CameraChoice, float ZRotate, float ForwardDisplacement, float LeftDisplacement, float VerticalDisplacement){
            this.XRotate = CameraOrientation;
            this.YRotate = CameraChoice;
            this.ZRotate = ZRotate;
            this.ForwardDisplacement = ForwardDisplacement;
            this.LeftDisplacement = LeftDisplacement;
            this.VerticalDisplacement = VerticalDisplacement;
            this.cameraDirection = cameraDirection;
        }
    }

}
