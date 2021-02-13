package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;

import org.opencv.core.Core;
import org.opencv.core.CvType;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.MatOfPoint2f;
import org.opencv.core.Point;
import org.opencv.core.RotatedRect;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.ArrayList;
import java.util.List;

@Config
public class RingPipeline extends OpenCvPipeline {
    public static double redMinThreshold = 170.186;
    public static double redMaxThreshold = 255;
    public static int dilateSize = 0;
    public static int erodeSize = 0;
    public static double cannyThreshold = 255;
    public static double sizeThreshold = 100;

    Mat output = new Mat();
    Mat dst = new Mat();
    Mat Cr = new Mat();
    Mat ycrcb = new Mat();
    Mat morph2 = new Mat();
    Mat morph1 = new Mat();
    Mat hierarchy = new Mat();
    Mat cannyOutput = new Mat();
    Mat otherOutput = new Mat();

    Mat dilate = Imgproc.getStructuringElement(Imgproc.CV_SHAPE_ELLIPSE, new Size(2 * dilateSize + 1, 2 * dilateSize + 1),
            new Point(dilateSize, dilateSize));
    Mat erode = Imgproc.getStructuringElement(Imgproc.CV_SHAPE_ELLIPSE, new Size(2 * erodeSize + 1, 2 * erodeSize + 1),
            new Point(erodeSize, erodeSize));

    enum Stage
    {
        YCrCb_CHAN2,
        THRESHOLD,
        DILATE,
        ERODE,
        CANNY,
        CONTOURS,
        RAW_IMAGE,
    }

    public static Stage stageToRenderToViewport = Stage.YCrCb_CHAN2;
    private Stage[] stages = Stage.values();

    @Override
    public void onViewportTapped()
    {
        /*
         * Note that this method is invoked from the UI thread
         * so whatever we do here, we must do quickly.
         */

        int currentStageNum = stageToRenderToViewport.ordinal();

        int nextStageNum = currentStageNum + 1;

        if(nextStageNum >= stages.length)
        {
            nextStageNum = 0;
        }

        stageToRenderToViewport = stages[nextStageNum];
    }

    @Override
    public Mat processFrame(Mat input) {
        Imgproc.cvtColor(input, ycrcb, Imgproc.COLOR_BGR2YCrCb);
        Core.extractChannel(ycrcb, Cr, 2);
        Imgproc.threshold(Cr, dst, redMinThreshold, redMaxThreshold, 3);
        int kernelSize = dilateSize;
        int kernelSize2 = erodeSize;
        Imgproc.erode(dst, morph1, erode);
        Imgproc.erode(morph1, morph1, erode);
        Imgproc.dilate(morph1, morph2, dilate);
        Imgproc.dilate(morph2, morph2, dilate);
//        int kernelSize3 = 1;
//        Mat element3 = Imgproc.getStructuringElement(Imgproc.CV_SHAPE_ELLIPSE, new Size(2 * kernelSize3 + 1, 2 * kernelSize3 + 1),
//                new Point(kernelSize3, kernelSize3));
//        Imgproc.morphologyEx(dst, dst, Imgproc.MORPH_GRADIENT,element3);
        Imgproc.Canny(morph2, cannyOutput, cannyThreshold, cannyThreshold * 2);
        List<MatOfPoint> contours = new ArrayList<>();
        Imgproc.findContours(morph2, contours, hierarchy, Imgproc.RETR_TREE, Imgproc.CHAIN_APPROX_NONE);
        input.copyTo(output);
        for(int i =0; i<contours.size(); i++){
            MatOfPoint2f f = new MatOfPoint2f();
            contours.get(i).convertTo(f, CvType.CV_32FC2);
            RotatedRect e = Imgproc.minAreaRect(f);
            Point[] pt = new Point[4];
            e.points(pt);
            List<MatOfPoint> bl = new ArrayList<>();
            bl.add(new MatOfPoint(pt));
            if(e.size.height+e.size.width>sizeThreshold && hierarchy.get(0,i)[3]==-1 && hierarchy.get(0, i)[2] != -1){
                Imgproc.ellipse(output,e,new Scalar(0,255,0),5);
                Imgproc.circle(output,e.center, 5, new Scalar(0, 255,0));
                Imgproc.drawContours(output, bl,-1, new Scalar(255,0,0));
//                            Imgproc.putText(input,e.size.width/e.size.height+"",pt[0],0,1.0,new Scalar(0,255,0));
                if(hierarchy.get(0, i)[2] != -1) {
                    MatOfPoint2f f2 = new MatOfPoint2f();
                    contours.get((int) hierarchy.get(0, i)[2]).convertTo(f2, CvType.CV_32FC2);
                    RotatedRect e2 = Imgproc.minAreaRect(f2);
                    Imgproc.ellipse(output, e2, new Scalar(0, 0, 255), 5);
                }

            }
            else if(e.size.height+e.size.width>sizeThreshold && hierarchy.get(0,i)[3]==-1){
                Imgproc.ellipse(output,e,new Scalar(0,255,0),5);
                Imgproc.circle(output,e.center, 5, new Scalar(0, 255,0));
                Imgproc.drawContours(output, bl,-1, new Scalar(0,0,255));
                Imgproc.putText(output,e.size.height+e.size.width+"",pt[0],0,1.0,new Scalar(0,255,0));
            }
        }

        switch(stageToRenderToViewport){
            case YCrCb_CHAN2:
                return Cr;
            case THRESHOLD:
                return dst;
            case ERODE:
                return morph1;
            case DILATE:
                return morph2;
            case CANNY:
                return cannyOutput;
            case CONTOURS:
                return output;
            default:
                return input;
        }

    }
}
