
package org.firstinspires.ftc.teamcode;

//import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.vision.VisionProcessor;
import java.util.Map;
//import org.firstinspires.ftc.robotcore.external.Telemetry;
import java.util.Arrays;
import java.util.ArrayList;
import java.util.EnumMap;
import java.util.List;
import java.util.Map;

import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfDouble;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
//import org.openftc.easyopencv.OpenCvCamera;
///import org.openftc.easyopencv.OpenCvCameraFactory;
//import org.openftc.easyopencv.OpenCvCameraRotation;
//import org.openftc.easyopencv.OpenCvInternalCamera;
//import org.openftc.easyopencv.OpenCvPipeline;
//import org.openftc.easyopencv.OpenCvWebcam;


import android.graphics.Canvas;

import org.firstinspires.ftc.robotcore.internal.camera.calibration.CameraCalibration;

//////////////////////////////////////////////////////////
//
// This class searches for colors within 3 user-defined regions.
// It returns data for each color, which allows the user to 
// prioritize colors if multiple colors are found in a region.
// It also can return which region has the most of a given color.
//
// It is a VisionProcessor that can be passed into the VisionPortal.
//
// Example:
//
// ColorVisionProcessor colorProcessor = new ColorVisionProcessor();
// VisionPortal visionPortal = new VisionPortal.easyCreateWithDefaults(hardwareMap.get(WebcamName.class, "Webcam 1"), colorProcessor);
// 
/////////////////////////////////////////////////////////

public class ColorVisionProcessor  implements VisionProcessor {

    public enum Color_Enum
    {
        Color_None,
        Color_Green,
        Color_Red,
        Color_Blue,
        Color_Yellow,
        Color_White
    }

    // this is used to return color information
    public class ColorData {
        public Color_Enum color = Color_Enum.Color_None;
        public double score;
        public int squareCount;
        public int x_location;
        public int y_location;
    }

    // this is a constant
    final int NumRegions = 3;

    // these can be configured at run time for where you
    // want your camera to look for colors
    public Point[] RegionTopLeft = new Point[NumRegions];
    public int[] RegionWidth = new int[NumRegions];
    public int[] RegionHeight = new int[NumRegions];

    // these can be configurated but usually don't need to change
    public final int SquareSize = 5;           // pixel size of edge of squares for checking
    public int MinSaturation = 100;
    public int MinBrightness = 75;
    public double MaxStdDev = 10;

    // constructor
    public ColorVisionProcessor() {

        // set default values which can be changed later
        RegionTopLeft[0] = new Point(109, 98);
        RegionTopLeft[1] = new Point(181, 98);
        RegionTopLeft[2] = new Point(253, 98);

        RegionWidth[0] = 60;
        RegionWidth[1] = 60;
        RegionWidth[2] = 60;
        RegionHeight[0] = 80;
        RegionHeight[1] = 80;
        RegionHeight[2] = 80;
    }

    private Scalar getColorScalar (Color_Enum color)
    {
        Scalar colorScalar = new Scalar(0, 0, 0);

        switch (color)
        {
            case Color_Blue :
                colorScalar = new Scalar(0, 0, 255);
                break;
            case Color_Green :
                colorScalar = new Scalar(0, 255, 0);
                break;
            case Color_Red :
                colorScalar = new Scalar(255, 0, 0);
                break;
            case Color_Yellow :
                colorScalar = new Scalar(255, 255, 0);
                break;
            case Color_White :
                colorScalar = new Scalar(255, 255, 255);
                break;
            default :
                break;
        }
        
        return colorScalar;
    }

    // returns the region number that has the most squares of the specified color.
    // Returns -1 if no region has the color.
    public int getRegion (Color_Enum color)
    {
        int bestRegion = -1;
        int maxSquares = 0;
        int squareCount = 0;
        Scalar colorScalar = getColorScalar (color);

        List<EnumMap<Color_Enum, ColorData>> colorData = getColorData();

        if (isCameraInitialized())
        {
            for (int region = 0; region < NumRegions; ++region) {

                if (colorData.get(region).get(color) != null)
                {
                   squareCount = colorData.get(region).get(color).squareCount;
                }

                if (squareCount > maxSquares)
                {
                   bestRegion = region;
                   maxSquares = squareCount;
                }
            }
        }

        return bestRegion;
    }

    public boolean isCameraInitialized() {
        boolean returnValue = false;
        List<EnumMap<Color_Enum, ColorData>> colorData = getColorData();

        // after the first frame is processed, each region will have data
        if (colorData.size() >= NumRegions) {
            returnValue = true;
        }
        return (returnValue);
    }

    public boolean isRegionBlue(int region) {
        List<EnumMap<Color_Enum, ColorData>> colorData = getColorData();
        return (colorData.get(region).get(Color_Enum.Color_Blue).color == Color_Enum.Color_Blue);
    }

    public boolean isRegionRed(int region) {
        List<EnumMap<Color_Enum, ColorData>> colorData = getColorData();
        return (colorData.get(region).get(Color_Enum.Color_Red).color == Color_Enum.Color_Red);
    }

    public boolean isRegionYellow(int region) {
        List<EnumMap<Color_Enum, ColorData>> colorData = getColorData();
        return (colorData.get(region).get(Color_Enum.Color_Yellow).color == Color_Enum.Color_Yellow);
    }

    public boolean isRegionGreen(int region) {
        List<EnumMap<Color_Enum, ColorData>> colorData = getColorData();
        return (colorData.get(region).get(Color_Enum.Color_Green).color == Color_Enum.Color_Green);
    }

    // this class is used for drawing boxes on the screen
    public class DebugData
    {
        public Color_Enum color;
        public Point upperLeft;
        public Point lowerRight;
        public int width;

        // constructor
        public DebugData ()
        {
        }

        public DebugData (Color_Enum color, Point upperLeft, Point lowerRight, int width)
        {
            // call common constructor
            this();

            this.color = color;
            this.upperLeft = upperLeft;
            this.lowerRight = lowerRight;
            this.width = width;
        }
    }

    // this class is to store telemetry data from camera thread
    public class TelemetryData {
        public String caption;
        public Object object;
    
        // constructor
        public TelemetryData (String caption, Object object) {
            this.caption = caption;
            this.object = object;
        }
    }

    // this class is used to pass color data from camera thread to opMode thread
    public class SynchronizedColorData {
        List<EnumMap<Color_Enum, ColorData>> colorData = new ArrayList<EnumMap<Color_Enum, ColorData>>();

        public synchronized void setColorData (List<EnumMap<Color_Enum, ColorData>> colorData) {
            // copy the list without being interrupted
            this.colorData = new ArrayList<EnumMap<Color_Enum, ColorData>>(colorData);
        }

        public synchronized List<EnumMap<Color_Enum, ColorData>> getColorData () {
            // get the list without being interrupted
            return this.colorData;
        }
    }

    // this class is used to pass telemetry data from camera thread to opMode thread
    public class SynchronizedTelemetryList {
        private List<TelemetryData> list = new ArrayList<TelemetryData>();
    
        public synchronized void setList (List<TelemetryData> list) {
            // copy the list without being interrupted
            this.list = new ArrayList<TelemetryData>(list);
        }

        public synchronized List<TelemetryData> getList () {
            // get the list without being interrupted
            return this.list;
        }
    }

        /*
         * Points which actually define the sample region rectangles, derived from above values
         *
         * Example of how points work to define a rectangle
         *
         *   ------------------------------------
         *   | (0,0) TopLeft                    |
         *   |                                  |
         *   |                                  |
         *   |                                  |
         *   |                                  |
         *   |                                  |
         *   |                                  |
         *   |              BottomRight (70,50) |
         *   ------------------------------------
         *
         */

        // Working variables

        List<EnumMap<Color_Enum, ColorData>> region_colorData = new ArrayList<EnumMap<Color_Enum, ColorData>>();
        private SynchronizedColorData synchronizedColorData = new SynchronizedColorData();

        List<DebugData> debugList = new ArrayList<DebugData>();

        private List<TelemetryData> telemetryData = new ArrayList<TelemetryData>();
        private SynchronizedTelemetryList synchronizedTelemetryData = new SynchronizedTelemetryList();

        /*
         * This function takes the RGB frame and converts to HSV
         */
        Mat inputToHSV(Mat input)
        {
            Mat hsv = new Mat();

            Imgproc.cvtColor(input, hsv, Imgproc.COLOR_RGB2HSV);
            
            return hsv;
        }

        //
        // finds the average hue and saturation of the input, and maps it to a color
        // with an associated score. Location is not set.
        //
        ColorData getColorData(Mat input)
        {
            ColorData colorData = new ColorData();

            int deltaHue = 999;
            int deltaSat = 999;

            colorData.color = Color_Enum.Color_None;
            colorData.score = 0.0;

            // channel 0 is hue
          
            // get average and standard deviation
            MatOfDouble mean = new MatOfDouble();
            MatOfDouble stdDev = new MatOfDouble();
            Core.meanStdDev(input, mean, stdDev);

            int average_hue = (int) mean.get(0, 0)[0];
            int stdDev_hue = (int) stdDev.get(0, 0)[0];

            // channel 1 is saturation
            int average_sat = (int) Core.mean(input).val[1];
            // channel 2 is brightness
            int average_brightness = (int) Core.mean(input).val[2];

            // since red is at 0/180, need to adjust to 90-270 with red at 180
            Mat redMask = new Mat();
            Mat redHSV = input.clone();
          
            // for redMask, find all values between 0-89
            Core.inRange(input, new Scalar(0,0,0), new Scalar(89,255,255), redMask);
            // convert them to 180-270 (by adding 180)
            Core.add(input, new Scalar(180,0,0), redHSV, redMask);

            // channel 0 is hue, with red at 180

            // get average and standard deviation
            mean = new MatOfDouble();
            stdDev = new MatOfDouble();
            Core.meanStdDev(redHSV, mean, stdDev);

            int average_hue_red = (int) mean.get(0, 0)[0];
            int stdDev_hue_red = (int) stdDev.get(0, 0)[0];

            // channel 1 is saturation
            int average_sat_red = (int) Core.mean(redHSV).val[1];

            // check hue and saturation to determine a color

            if (average_hue > 90 && average_hue < 120 && average_sat > MinSaturation && average_brightness > MinBrightness && stdDev_hue <= MaxStdDev)
            {
                colorData.color = Color_Enum.Color_Blue;
                deltaHue = 105 - average_hue;
                deltaSat = 150 - average_sat;
            }
            if (average_hue > 45 && average_hue < 85 && average_sat > MinSaturation && average_brightness > MinBrightness && stdDev_hue <= MaxStdDev)
            {
                colorData.color = Color_Enum.Color_Green;
                deltaHue = 75 - average_hue;
                deltaSat = 150 - average_sat;
            }
            if (average_hue > 20 && average_hue < 40 && average_sat > MinSaturation && average_brightness > MinBrightness && stdDev_hue <= MaxStdDev)
            {
                colorData.color = Color_Enum.Color_Yellow;
                deltaHue = 30 - average_hue;
                deltaSat = 150 - average_sat;
            }
            if (average_hue_red > 170 && average_hue_red < 190 && average_sat_red > MinSaturation && average_brightness > MinBrightness && stdDev_hue_red <= MaxStdDev)
            {
                colorData.color = Color_Enum.Color_Red;
                deltaHue = 180 - average_hue_red;
                deltaSat = 150 - average_sat_red;
            }

            // compute score
            if (deltaSat < 0) {
                deltaSat = 0;
            }
          
            colorData.score = 100 - Math.sqrt(deltaHue * deltaHue + deltaSat * deltaSat);
          
            return colorData;
        }

        ColorData searchForColor(Mat input, Color_Enum color, int region) {

            ColorData bestColorData = new ColorData();
            ColorData workingColorData = new ColorData();
            DebugData bestDebug;
            Mat regionMat;
            
            bestColorData.squareCount = 0;

            if (region < NumRegions)
            {
                Point regionTopLeft = new Point(
                    RegionTopLeft[region].x,
                    RegionTopLeft[region].y);
                Point regionBottomRight = new Point(
                    RegionTopLeft[region].x + RegionWidth[region],
                    RegionTopLeft[region].y + RegionHeight[region]);

                regionMat = input.submat(new Rect(regionTopLeft, regionBottomRight));

                //
                // split the region into 5x5 pixel squares and get the color in each square
                //

                // search in squares for highest scoring one
                for (int row = 0; row < regionMat.rows() - SquareSize; row += SquareSize)
                {
                    for (int column = 0; column < regionMat.cols() - SquareSize; column += SquareSize)
                    {
                        Mat square = regionMat.submat(row, row + SquareSize, column, column + SquareSize);

                        int row2 = row + (int)RegionTopLeft[region].y;
                        int col2 = column + (int)RegionTopLeft[region].x;
                        Point upperLeft = new Point(col2, row2);
                        Point lowerRight = new Point(col2 + SquareSize, row2 + SquareSize);

                        workingColorData = getColorData(square);

                        int width = 1;
                        if (workingColorData.score >= 98)
                        {
                            width = 2;
                        }

                        if (workingColorData.color == color)
                        {
                            bestColorData.squareCount++;
                  
                            DebugData data = new DebugData(color, upperLeft, lowerRight, width);
                            debugList.add(data);

                            if (workingColorData.score > bestColorData.score)
                            {
                                bestColorData.color = workingColorData.color;
                                bestColorData.score = workingColorData.score;
                                bestColorData.x_location = col2;
                                bestColorData.y_location = row2;
                                bestDebug = data;
                            }
                        }
                    }
                }
            }
            
            return bestColorData;
        }

    @Override
    public void init(int width, int height, CameraCalibration calibration)
    {
        // initialize static color list
        DebugData debugData = new DebugData ();
    }
//    Object processFrame(Mat frame, long captureTimeNanos);

    @Override
    public void onDrawFrame(Canvas canvas, int onscreenWidth, int onscreenHeight, float scaleBmpPxToCanvasPx, float scaleCanvasDensity, Object userContext)
    {
    
    }

    @Override
    public Mat processFrame(Mat input, long captureTimeNanos)
    {
        Mat hsvMat = new Mat();
        MatOfDouble mean = new MatOfDouble();
        MatOfDouble stdDev = new MatOfDouble();
        ColorData colorData = new ColorData();

        // clear out previous data
        telemetryData.clear();
        debugList.clear();
        for (int region = 0; region < NumRegions; ++region) {
            region_colorData.add(region, new EnumMap<Color_Enum, ColorData>(Color_Enum.class));
        }

        // convert to HSV format
        hsvMat = inputToHSV(input);

        for (int region = 0; region < NumRegions; ++region) {
            // search for the color
            colorData = searchForColor(hsvMat, Color_Enum.Color_Red, region);
telemetryData.add (new TelemetryData("Red score", colorData.score));
            // save off the data
            region_colorData.get(region).put(Color_Enum.Color_Red, colorData);

            colorData = searchForColor(hsvMat, Color_Enum.Color_Blue, region);
telemetryData.add (new TelemetryData("Blue score", colorData.score));
            region_colorData.get(region).put(Color_Enum.Color_Blue, colorData);

            colorData = searchForColor(hsvMat, Color_Enum.Color_Green, region);
            region_colorData.get(region).put(Color_Enum.Color_Green, colorData);

            colorData = searchForColor(hsvMat, Color_Enum.Color_Yellow, region);
telemetryData.add (new TelemetryData("Yellow score", colorData.score));
            region_colorData.get(region).put(Color_Enum.Color_Yellow, colorData);
        }
//            findBestColor(hsv, new Point(0,0), colorData);
//getColorData(region1_hsv, colorData);

telemetryData.add (new TelemetryData("Red region", getRegion (Color_Enum.Color_Red)));
telemetryData.add (new TelemetryData("Blue region", getRegion (Color_Enum.Color_Blue)));


        /*
         * Draw a rectangle showing each region on the screen.
         * Simply a visual aid. Serves no functional purpose.
         */
        Scalar WHITE = new Scalar (255, 255, 255);
        DebugData debugData = new DebugData();
        
        for (int region = 0; region < NumRegions; ++region)
        {
            Color_Enum color = Color_Enum.Color_White;

            if (region_colorData.get(region).get(Color_Enum.Color_Green).score > 0) {
telemetryData.add (new TelemetryData("Green score", region_colorData.get(region).get(Color_Enum.Color_Green).score));
                color = Color_Enum.Color_Green;
            }
            else if (region_colorData.get(region).get(Color_Enum.Color_Yellow).color == Color_Enum.Color_Yellow) {
telemetryData.add (new TelemetryData("Yellow score", region_colorData.get(region).get(Color_Enum.Color_Yellow).score));
                color = Color_Enum.Color_Yellow;
            }
            else if (region_colorData.get(region).get(Color_Enum.Color_Red).score > 0) {
                color = Color_Enum.Color_Red;
            }
            else if (region_colorData.get(region).get(Color_Enum.Color_Blue).score > 0) {
                color = Color_Enum.Color_Blue;
            }
telemetryData.add (new TelemetryData("Color", color));

            Point regionBottomRight = new Point(RegionTopLeft[region].x + RegionWidth[region],
                                                RegionTopLeft[region].y + RegionHeight[region]);

            debugList.add(new DebugData(color, RegionTopLeft[region], regionBottomRight, 2));
        }

        synchronizedColorData.setColorData (region_colorData);
        synchronizedTelemetryData.setList (telemetryData);

telemetryData.clear();
        // draw debug data on the screen
        Mat output = input;
        for (DebugData data : debugList)
        {
            Scalar color = getColorScalar (data.color);

            Imgproc.rectangle(output, data.upperLeft, data.lowerRight, color, data.width);
        }

        return output;
    }

    public List<EnumMap<Color_Enum, ColorData>> getColorData()
    {
        return synchronizedColorData.getColorData();
    }

    public List<TelemetryData> getTelemetryData()
    {
        return synchronizedTelemetryData.getList();
    }

}
