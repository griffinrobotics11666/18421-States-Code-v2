package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;

import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint2f;
import org.opencv.core.Point;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

@Config
public class IPMPipeline extends OpenCvPipeline {
    public Mat transformationMatrix;
    /*
    Coordinates of some flat rectangular object in front of the camera on the field
    Place this rectangular object such that its bottom lines up with the bottom of the frame and it is centered width-wise
     */
    public static Point tl = new Point(0,0);
    public static Point tr = new Point(0,0);
    public static Point bl = new Point(0,0);
    public static Point br = new Point(0,0);

    //Size of that rectangular object in inches
    public static Size objectSize = new Size(0,0);

    //This is to convert inches to pixels (e.g, if scale=2.0, then an object that is 12 inches long will be 24 pixels long in the destination image)
    public static double scale = 1.0;

    public void getTransformationMatrix(Size size) {
        MatOfPoint2f src = new MatOfPoint2f(tl, tr, bl, br);
        MatOfPoint2f dst = new MatOfPoint2f(
                new Point((size.width/2)-(objectSize.width*scale/2),size.height-(objectSize.height*scale)),
                new Point((size.width/2)+(objectSize.width*scale/2),size.height-(objectSize.height*scale)),
                new Point((size.width/2)-(objectSize.width*scale/2), size.height),
                new Point((size.width/2)+(objectSize.width*scale/2), size.height)
                );
        transformationMatrix = Imgproc.getPerspectiveTransform(src, dst);
    }

    public Mat transformImage(Mat src) {
        Mat dst = new Mat();
        getTransformationMatrix(src.size());
        Imgproc.warpPerspective(src, dst, transformationMatrix, src.size(), Imgproc.INTER_LINEAR);
        return dst;
    }

    @Override
    public Mat processFrame(Mat input) {
        return transformImage(input);
    }
}
