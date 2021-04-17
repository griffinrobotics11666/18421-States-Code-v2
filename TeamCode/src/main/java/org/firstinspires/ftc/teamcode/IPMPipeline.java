package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;

import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.MatOfPoint2f;
import org.opencv.core.Point;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.ArrayList;
import java.util.List;

@Config
public class IPMPipeline extends OpenCvPipeline {
    public Mat transformationMatrix;
    /*
    Coordinates of some flat rectangular object (like a piece of paper) in front of the camera on the field
    Place this rectangular object such that its bottom lines up with the bottom of the frame and it is centered width-wise
     */
    public static Point tl = new Point(0,0);
    public static Point tr = new Point(0,0);
    public static Point bl = new Point(0,0);
    public static Point br = new Point(0,0);

    //This is to correct for any errors in the object's position. It doesn't correct for any rotation errors, so make sure that the object is straight.
    public static Point offset = new Point(0,0);

    //Size of that rectangular object in inches
    public static Size objectSize = new Size(0,0);

    //This is to convert inches to pixels (e.g, if scale=2.0, then an object that is 12 inches long will be 24 pixels long in the destination image)
    public static double scale = 1.0;

    public enum Mode{
        FRAME,
        WARPED
    }
    public static Mode mode = Mode.FRAME;


    public void getTransformationMatrix(Size size) {
        MatOfPoint2f src = new MatOfPoint2f(tl, tr, bl, br);
        MatOfPoint2f dst = new MatOfPoint2f(
                new Point((size.width/2+offset.x)-(objectSize.width*scale/2),size.height+offset.y-(objectSize.height*scale)),
                new Point((size.width/2+offset.x)+(objectSize.width*scale/2),size.height+offset.y-(objectSize.height*scale)),
                new Point((size.width/2+offset.x)-(objectSize.width*scale/2), size.height+offset.y),
                new Point((size.width/2+offset.x)+(objectSize.width*scale/2), size.height+offset.y)
        );
        transformationMatrix = Imgproc.getPerspectiveTransform(src, dst);
    }

    public Mat transformImage(Mat src) {
        Mat dst = new Mat();
        getTransformationMatrix(src.size());
        Imgproc.warpPerspective(src, dst, transformationMatrix, src.size(), Imgproc.INTER_LINEAR);
        return dst;
    }

    public Mat drawBorder(Mat src){
        MatOfPoint dst2 = new MatOfPoint(
                tl,
                tr,
                br,
                bl
        );
        List<MatOfPoint> points = new ArrayList<>();
        points.add(dst2);
        Imgproc.polylines(src, points, true, new Scalar(0,255,0));
        return src;
    }

    @Override
    public Mat processFrame(Mat input) {
        switch(mode){
            case FRAME:
                return drawBorder(input);
            case WARPED:
                return transformImage(input);
        }
        return null;
    }
}
