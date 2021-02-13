package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;

/*
 * This sample demonstrates how to stream frames from Vuforia to the dashboard. Make sure to fill in
 * your Vuforia key below and select the 'Camera' preset on top right of the dashboard. This sample
 * also works for UVCs with slight adjustments.
 */
@Autonomous
public class VuforiaStreamOpMode extends LinearOpMode {

    // TODO: fill in
    public static final String VUFORIA_LICENSE_KEY = "AdYYXLj/////AAABmbrz6/MNLUKlnU5JIPwkiDQ5jX+GIjfuIEgba3irGu46iS/W1Q9Z55uLSl31zGtBX3k5prkoSK6UxLR9gyvyIwSzRe2FOFGHEvJ19uG+pqiJJfkaRb0mCUkrx4U/fH6+Agp+7lOHB8IYjziNSuBMgABbrii5tAQiXOGfGojY+IQ/enBoy+zWiwVBx9cPRBsEHu+ipK6RXQe7CeODCRN8anBfAsn5b2BoO9lcGE0DgZdRysyByQ4wuwNQxKjba18fnzSDWpm12Brx3Ao1vkGYxTyLQfsON5VotphvWwoZpoyD+Iav/yQmOxrQDBLox6SosF8jqG9sUC5LAAdiRIWr6sNRrGzeCtsHSJBplHboPMB3";


    @Override
    public void runOpMode() throws InterruptedException {
        // gives Vuforia more time to exit before the watchdog notices
        msStuckDetectStop = 2500;

        WebcamName webcamName = null;
        webcamName = hardwareMap.get(WebcamName.class, "webcam");
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        VuforiaLocalizer.Parameters vuforiaParams = new VuforiaLocalizer.Parameters(cameraMonitorViewId);
        vuforiaParams.vuforiaLicenseKey = VUFORIA_LICENSE_KEY;
        vuforiaParams.cameraDirection = VuforiaLocalizer.CameraDirection.BACK;
        vuforiaParams.cameraName=webcamName;
        VuforiaLocalizer vuforia = ClassFactory.getInstance().createVuforia(vuforiaParams);

        FtcDashboard.getInstance().startCameraStream(vuforia, 0);

        waitForStart();

        while (opModeIsActive());
    }
}