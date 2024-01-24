package org.firstinspires.ftc.teamcode.robonauts.vision;


import android.graphics.Bitmap;
import android.graphics.Canvas;
import android.util.Size;

import org.firstinspires.ftc.robotcore.external.function.Consumer;
import org.firstinspires.ftc.robotcore.external.function.Continuation;
import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.stream.CameraStreamSource;
import org.firstinspires.ftc.robotcore.internal.camera.calibration.CameraCalibration;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.VisionProcessor;
import org.opencv.android.Utils;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;

import java.util.concurrent.atomic.AtomicReference;

public class RedPropThreshold implements VisionProcessor, CameraStreamSource {
    Mat testMat = new Mat();
    Mat highMat = new Mat();
    Mat lowMat = new Mat();
    Mat finalMat = new Mat();
    double redThreshold = 0.5;
    private final AtomicReference<Bitmap> lastFrame = new AtomicReference<>(Bitmap.createBitmap(1, 1, Bitmap.Config.RGB_565));

    String outStr = "left"; //Set a default value in case vision does not work
    public enum Location {
        LEFT,
        CENTER,
        RIGHT,
        NOT_FOUND
    }
    public static double percentThreshold = 0.2;

    static final Rect LEFT_RECTANGLE = new Rect(
            new Point(0, 0),
            new Point(0, 0)
    );
    Scalar colorEmpty = new Scalar(0, 0, 0);
    Scalar colorProp = new Scalar(0, 0, 255);

    static final Rect LEFT_ROI = new Rect(
            new Point(10, 300),
            new Point(190, 415)
    );
    static final Rect CENTER_ROI = new Rect(
            new Point(110, 165),
            new Point(220, 210)
    );
    static final Rect RIGHT_ROI = new Rect(
            new Point(490, 180),
            new Point(600, 225)
    );
    static final Rect RIGHT_RECTANGLE = new Rect(
            new Point(0, 0),
            new Point(0, 0)
    );
    Location location = Location.NOT_FOUND;

    @Override
    public void init(int width, int height, CameraCalibration calibration) {
        lastFrame.set(Bitmap.createBitmap(width, height, Bitmap.Config.RGB_565));

    }

    @Override
    public Object processFrame(Mat frame, long captureTimeNanos) {

        if (frame == null) {
            location = Location.NOT_FOUND;
            return Location.NOT_FOUND;
        }
        Imgproc.cvtColor(frame, testMat, Imgproc.COLOR_RGB2HSV);


        Scalar lowHSVRedLower = new Scalar(0, 100, 20);  //Beginning of Color Wheel
        Scalar lowHSVRedUpper = new Scalar(10, 255, 255);

        Scalar redHSVRedLower = new Scalar(160, 100, 20); //Wraps around Color Wheel
        Scalar highHSVRedUpper = new Scalar(180, 255, 255);

        Core.inRange(testMat, lowHSVRedLower, lowHSVRedUpper, lowMat);
        Core.inRange(testMat, redHSVRedLower, highHSVRedUpper, highMat);

        testMat.release();

        Core.bitwise_or(lowMat, highMat, finalMat);

        Mat left = testMat.submat(LEFT_ROI);
        Mat center = testMat.submat(CENTER_ROI);
        Mat right = testMat.submat(RIGHT_ROI);

        double leftRaw = Core.sumElems(left).val[0];
        double centerRaw = Core.sumElems(center).val[0];
        double rightRaw = Core.sumElems(right).val[0];

        double leftValue = leftRaw / LEFT_ROI.area() / 255;
        double centerValue = centerRaw / CENTER_ROI.area() / 255;
        double rightValue = rightRaw / RIGHT_ROI.area() / 255;


        lowMat.release();
        highMat.release();
        left.release();
        center.release();
        right.release();
        Location currLocation = Location.NOT_FOUND;

        if (leftValue > centerValue && leftValue > rightValue && leftValue > percentThreshold) {
            currLocation =  Location.LEFT;
        } else if (centerValue > leftValue && centerValue > rightValue && centerValue > percentThreshold) {
            currLocation = Location.CENTER;
        } else if (rightValue > leftValue && rightValue > centerValue && rightValue > percentThreshold) {
            currLocation = Location.RIGHT;
        } else {
            currLocation = Location.NOT_FOUND;
        }

       /* telemetry.addData("Found prop at", currLocation);
        telemetry.addData("Detected left", leftValue);
        telemetry.addData("Detected center", centerValue);
        telemetry.addData("Detected right", rightValue);
        telemetry.update();*/

        Imgproc.cvtColor(testMat, testMat, Imgproc.COLOR_GRAY2RGB);

        Imgproc.rectangle(frame, LEFT_ROI, currLocation == Location.LEFT ? colorProp : colorEmpty);
        Imgproc.rectangle(frame, CENTER_ROI, currLocation == Location.CENTER ? colorProp : colorEmpty);
        Imgproc.rectangle(frame, RIGHT_ROI, currLocation == Location.RIGHT ? colorProp : colorEmpty);

        Bitmap b = Bitmap.createBitmap(frame.width(), frame.height(), Bitmap.Config.RGB_565);
        Utils.matToBitmap(frame, b);
        lastFrame.set(b);

        double leftBox = Core.sumElems(finalMat.submat(LEFT_RECTANGLE)).val[0];
        double rightBox = Core.sumElems(finalMat.submat(RIGHT_RECTANGLE)).val[0];

        double averagedLeftBox = leftBox / LEFT_RECTANGLE.area() / 255;
        double averagedRightBox = rightBox / RIGHT_RECTANGLE.area() / 255; //Makes value [0,1]


        if(averagedLeftBox > redThreshold){        //Must Tune Red Threshold
            outStr = "left";
        }else if(averagedRightBox> redThreshold){
            outStr = "center";
        }else{
            outStr = "right";
        }

        finalMat.copyTo(frame);
        location = currLocation;
        return currLocation;
        //return null;            //You do not return the original mat anymore, instead return null

    }


    @Override
    public void onDrawFrame(Canvas canvas, int onscreenWidth, int onscreenHeight, float scaleBmpPxToCanvasPx, float scaleCanvasDensity, Object userContext) {

    }

    public String getPropPosition(){  //Returns postion of the prop in a String
        return location.name();
    }


    @Override
    public void getFrameBitmap(Continuation<? extends Consumer<Bitmap>> continuation) {
        continuation.dispatch(bitmapConsumer -> bitmapConsumer.accept(lastFrame.get()));

    }
}
