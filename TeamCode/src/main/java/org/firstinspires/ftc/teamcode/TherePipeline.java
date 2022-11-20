package org.firstinspires.ftc.teamcode;

import org.firstinspires.ftc.teamcode.EasyOpenCVExamples.PipelineStageSwitchingExample;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

public class TherePipeline extends OpenCvPipeline {
    private static final Scalar BLUE = new Scalar (0,0,255);
    private static final Scalar GREEN = new Scalar (0,255,0);
    private static final Scalar RED = new Scalar (255,0,0);
    public enum PowerPlayPosition {
        LEFT,
        CENTER,
        RIGHT
    }

    boolean viewportPaused = false;
    static final Point THEREBOX_TOPRIGHT_ANCHOR_POINT = new Point (200,110);
    static final int THEREBOX_WIDTH = 40;
    static final int THEREBOX_HEIGHT = 40;//was 20
    public String color = "RED";
    PowerPlayPosition coneState = PowerPlayPosition.LEFT; //default


    Point THEREBOX_point2 = new Point (
            THEREBOX_TOPRIGHT_ANCHOR_POINT.x + THEREBOX_WIDTH,
            THEREBOX_TOPRIGHT_ANCHOR_POINT.y + THEREBOX_HEIGHT);

    Mat THEREBOX_B;
    Mat RGB = new Mat();
    Mat B = new Mat();
    int avgB;

    Mat THEREBOX_G;
    Mat G = new Mat();
    int avgG;

    Mat THEREBOX_R;
    Mat R = new Mat();
    int avgR;

    void splitChannels(Mat input)
    {
        Core.extractChannel(input,R,0);
        Core.extractChannel(input,G,1);
        Core.extractChannel(input,B,2);
    }
    private volatile PowerPlayPosition WHERE = PowerPlayPosition.LEFT; //default = LEFT [hi this code isn't disgusting shut up (respectfully) jk lol]

    @Override
    public void init(Mat firstImage){
        splitChannels(firstImage);

        THEREBOX_R = R.submat(new Rect(THEREBOX_TOPRIGHT_ANCHOR_POINT, THEREBOX_point2));
        THEREBOX_G = G.submat(new Rect(THEREBOX_TOPRIGHT_ANCHOR_POINT, THEREBOX_point2));
        THEREBOX_B = B.submat(new Rect(THEREBOX_TOPRIGHT_ANCHOR_POINT, THEREBOX_point2));
    }
    /*
     * NOTE: if you wish to use additional Mat objects in your processing pipeline, it is
     * highly recommended to declare them here as instance variables and re-use them for
     * each invocation of processFrame(), rather than declaring them as new local variables
     * each time through processFrame(). This removes the danger of causing a memory leak
     * by forgetting to call mat.release(), and it also reduces memory pressure by not
     * constantly allocating and freeing large chunks of memory.
     */




    @Override
    public Mat processFrame(Mat input){
        /* wow we're so cool */

        splitChannels(input);
        int[] avgs = new int[3];

        avgR = (int) Core.mean(THEREBOX_R).val[0];
        avgG = (int) Core.mean(THEREBOX_G).val[0];
        avgB = (int) Core.mean(THEREBOX_B).val[0];

        avgs[0] = avgR;
        avgs[1] = avgG;
        avgs[2] = avgB;

        /*
         * IMPORTANT NOTE: the input Mat that is passed in as a parameter to this method
         * will only dereference to the same image for the duration of this particular
         * invocation of this method. That is, if for some reason you'd like to save a copy
         * of this particular frame for later use, you will need to either clone it or copy
         * it to another Mat.
         */

        /*
         * Draw a simple box around the middle 1/2 of the entire frame
         */

        Scalar thereColor = RED;

        int big1 = Math.max(avgR,avgG);
        int big2 = Math.max(big1,avgB);

        if (big2 == avgR){
            thereColor = RED;
            color = "RED";
            coneState = PowerPlayPosition.LEFT;

        }
        else if (big2 == avgG) {
            thereColor = GREEN;
            color = "GREEN";
            coneState = PowerPlayPosition.RIGHT;

        }
        else {
            thereColor = BLUE;
            color = "BLUE";
            coneState = PowerPlayPosition.CENTER;

        }


        Imgproc.rectangle(
                input,
                THEREBOX_TOPRIGHT_ANCHOR_POINT, THEREBOX_point2,
                thereColor , 2);

        /*
        Imgproc.rectangle(
                input,
                new Point(
                        150,
                        110),
                new Point(
                        170,
                        130),
                thereColor , 2);
               */


        /**
         * NOTE: to see how to get data from your pipeline to your OpMode as well as how
         * to change which stage of the pipeline is rendered to the viewport when it is
         * tapped, please see {@link PipelineStageSwitchingExample}
         */

        return input;
    }
    public PowerPlayPosition getAnalysis (){
        return coneState;
    }
    /*
    @Override
    public void onViewportTapped() {
      /*
       * The viewport (if one was specified in the constructor) can also be dynamically "paused"
       * and "resumed". The primary use case of this is to reduce CPU, memory, and power load
       * when you need your vision pipeline running, but do not require a live preview on the
       * robot controller screen. For instance, this could be useful if you wish to see the live
       * camera preview as you are initializing your robot, but you no longer require the live
       * preview after you have finished your initialization process; pausing the viewport does
       * not stop running your pipeline.
       *
       * Here we demonstrate dynamically pausing/resuming the viewport when the user taps it


      viewportPaused = !viewportPaused;

      if (viewportPaused) {
        webcam.pauseViewport();
      } else {
        webcam.resumeViewport();
      }
    }
    */
}
