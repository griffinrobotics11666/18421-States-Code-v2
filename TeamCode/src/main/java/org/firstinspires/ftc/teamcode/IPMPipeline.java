package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Vector2d;

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
    public static Point tl = new Point(90,150);
    public static Point tr = new Point(212,155);
    public static Point bl = new Point(58,240);
    public static Point br = new Point(256,240);
    
    //This is to correct for any positional offsets (not rotational though - make sure it's straight)
    public static Point offset = new Point(0,0);

    //This is the distance from the center of the robot to the bottom center of the camera frame in inches
    public static Point positionOffset = new Point(0, 0);

    //Size of that rectangular object in inches
    public static Size objectSize = new Size(8.5,11);

    //This is to convert inches to pixels (e.g, if scale=2.0, then an object that is 12 inches long will be 24 pixels long in the destination image)
    public static double scale = 5.0;

    public static Point testPoint = new Point(0,0);

    private enum Mode{
        FRAME,
        WARPED
    }
    public static Mode mode = Mode.FRAME;

    private MatOfPoint2f src;
    private MatOfPoint2f dst;
    private Mat dst2 = new Mat();
    private Mat point;

    public void getTransformationMatrix(Size size) {
        src = new MatOfPoint2f(tl, tr, bl, br);
        dst = new MatOfPoint2f(
                new Point((size.width/2 + offset.x)-(objectSize.width*scale/2),size.height + offset.y -(objectSize.height*scale)),
                new Point((size.width/2 + offset.x)+(objectSize.width*scale/2),size.height + offset.y -(objectSize.height*scale)),
                new Point((size.width/2 + offset.x)-(objectSize.width*scale/2), size.height + offset.y),
                new Point((size.width/2 + offset.x)+(objectSize.width*scale/2), size.height + offset.y)
                );
        transformationMatrix = Imgproc.getPerspectiveTransform(src, dst);
    }

    public Mat transformImage(Mat src) {
        getTransformationMatrix(src.size());
        Imgproc.warpPerspective(src, dst, transformationMatrix, src.size(), Imgproc.INTER_LINEAR);
        return dst;
    }

    public Mat drawBoundingBox(Mat src){
        List<MatOfPoint> list = new ArrayList<>();
        list.add(new MatOfPoint(
                tl,
                tr,
                bl,
                br
        ));
        Imgproc.polylines(src, list, true, new Scalar(0,255,0));
        Imgproc.putText(src, getPosition(testPoint, src.size()).toString(), testPoint, Imgproc.FONT_HERSHEY_PLAIN, 2, new Scalar(255,0,0));
        Imgproc.circle(src, testPoint, 1, new Scalar(255,0,0), 2);
        return src;
    }

    public Vector2d getPosition(Point src, Size size){
        point = new Mat();
        point.put(0,0,src.x,src.y);
        point = transformationMatrix.inv().mul(point);
        return new Vector2d(point.get(0,0)[0], point.get(0,0)[1]);
    }

    @Override
    public Mat processFrame(Mat input) {
        switch(mode){
            case FRAME:
                return drawBoundingBox(input);
            case WARPED:
                return transformImage(input);
        }
        return null;
    }
}
