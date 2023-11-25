/* ColorVisionProcessor.java - FTC VisionProcessor using OpenCV
 * Copyright (C) 2023 LAtimes2
 *
 * MIT License
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission
 * notice shall be included in all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 */

package org.firstinspires.ftc.teamcode;

import org.firstinspires.ftc.vision.VisionProcessor;
import java.util.Map;
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
// To define the regions, you can see them on the Driver Station during the
// init phase by selecting Camera Stream from the upper right menu.
//
// Example:
//
//      ColorVisionProcessor colorProcessor = new ColorVisionProcessor();
//      VisionPortal visionPortal = new VisionPortal.easyCreateWithDefaults(hardwareMap.get(WebcamName.class, "Webcam 1"), colorProcessor);
// 
//      // example for region 0. repeat for regions 1 and 2
//      colorProcessor.RegionTopLeft[0] = new Point(109, 98);
//      colorProcessor.RegionWidth[0] = 60;
//      colorProcessor.RegionHeight[0] = 80;

// For a game like PowerPlay, you may want to know if a color is in a region:
//
//      if (colorProcessor.isRegionGreen(1)) {
//          green = true;
//      }
//
// For a game like FreightFrenzy, you may want to know which region is yellow:
//
//      int region = colorProcessor.getRegion(ColorVisionProcessor.Color_Enum.Color_Yellow));
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

        // For webcam, screen size is (0,0) to (639,479)

        // you would change these to define the regions needed
        // for this year's game
        RegionTopLeft[0] = new Point(109, 98);
        RegionWidth[0] = 60;
        RegionHeight[0] = 80;

        RegionTopLeft[1] = new Point(181, 98);
        RegionWidth[1] = 60;
        RegionHeight[1] = 80;

        RegionTopLeft[2] = new Point(253, 98);
        RegionWidth[2] = 60;
        RegionHeight[2] = 80;
    }

    // returns the region number that has the most area of the specified color.
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

    // this returns true when the VisionProcessor is running. It should be checked prior to
    // calling waitForStart.
    public boolean isCameraInitialized() {
        boolean returnValue = false;
        List<EnumMap<Color_Enum, ColorData>> colorData = getColorData();

        // after the first frame is processed, each region will have data
        if (colorData.size() >= NumRegions) {
            returnValue = true;
        }
        return (returnValue);
    }

    // these functions indicate if the predominant color in the region is the color specified
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

    // if you need more detailed data that you can't get from the functions
    // above, this will provide it.
    public List<EnumMap<Color_Enum, ColorData>> getColorData()
    {
        return synchronizedColorData.getColorData();
    }

    // this is used to return telemetry data
    public class TelemetryData {
        public String caption;
        public Object object;
    
        // constructor
        public TelemetryData (String caption, Object object) {
            this.caption = caption;
            this.object = object;
        }
    }

    // this allows you to print telemetry data from the VisionProcessor in your OpMode
    public List<TelemetryData> getTelemetryData()
    {
        // To print the VisionProcessor telemetry data, add the following to your OpMode:
        //
        //      List<ColorVisionProcessor.TelemetryData> telemetryList = colorProcessor.getTelemetryData();
        //      for (ColorVisionProcessor.TelemetryData data : telemetryList)
        //      {
        //          telemetry.addData(data.caption, data.object);
        //      }
        //      telemetry.update();

        return synchronizedTelemetryData.getList();
    }

    // this class is used for drawing boxes on the screen
    private class DebugData
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

    // this class is used to pass color data from camera thread to opMode thread
    private class SynchronizedColorData {
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
    private class SynchronizedTelemetryList {
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
    *   |            BottomRight (639,479) |
    *   ------------------------------------
    *
    * A Mat is a 2-dimensional array of pixel values. A selected pixel can have multiple
    * values, so in RGB format value 0 is red, value 1 is green, and value 2 is blue.
    * Pixels can also be in HSV format, where value 0 is hue, value 1 is saturation, and
    * value 2 is value (brightness).
    *
    * For example, to get pixel (10, 20) blue value from an RGB Mat called input:
    *    int blueValue = input.get(10, 20)[2];
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
    ColorData computeColorData(Mat input)
    {
        // input is the pixels for a 5x5 square from the camera in HSV (Hue/Saturation/Value) format
    
        ColorData colorData = new ColorData();

        int deltaHue = 999;
        int deltaSat = 999;
        boolean done = false;
        int average_hue = 0;
        int average_sat = 0;
        int stdDev_hue = 0;

        // clear the color data
        colorData.color = Color_Enum.Color_None;
        colorData.score = 0.0;

        // Each pixel has 3 values:
        // channel 0 is hue
        // channel 1 is saturation
        // channel 2 is value/brightness

        // if not bright enough, we are done (color = none)
        //     Core.mean returns the mean (average) value of channel 2, which is value/brightness
        int average_brightness = (int) Core.mean(input).val[2];
        
        if (average_brightness < MinBrightness)
        {
            done = true;
        }

        if (!done)
        {
            // if not enough saturation, we are done (color = none)
            // A low saturation means the color is too close to white
            //     Core.mean returns the mean (average) value of channel 1, which is saturation
            average_sat = (int) Core.mean(input).val[1];

            if (average_sat < MinSaturation)
            {
                done = true;
            }
        }

        if (!done)
        {
            // get average and standard deviation
            MatOfDouble mean = new MatOfDouble();
            MatOfDouble stdDev = new MatOfDouble();

            // computes a 1x1 mat with 1 channel for mean and standard deviation
            Core.meanStdDev(input, mean, stdDev);

            average_hue = (int) mean.get(0, 0)[0];
            stdDev_hue = (int) stdDev.get(0, 0)[0];

            // if standard deviation is too high, that means it is not a solid
            // color and has too many different colors in it
            if (stdDev_hue >= MaxStdDev)
            {
                done = true;
            }
            
            // if it may be red, need to continue to do special processing
            // for standard deviation (since it crosses 0/180)
            if (average_hue > 170 || average_hue < 10)
            {
                done = false;
            }
        }

        if (!done)
        {
            // check hue to determine a color
            if (average_hue > 90 && average_hue < 120)
            {
                colorData.color = Color_Enum.Color_Blue;
                deltaHue = 105 - average_hue;
                deltaSat = 150 - average_sat;
            }
            else if (average_hue > 45 && average_hue < 85)
            {
                colorData.color = Color_Enum.Color_Green;
                deltaHue = 75 - average_hue;
                deltaSat = 150 - average_sat;
            }
            else if (average_hue > 20 && average_hue < 40)
            {
                colorData.color = Color_Enum.Color_Yellow;
                deltaHue = 30 - average_hue;
                deltaSat = 150 - average_sat;
            }
            else
            {
                // since red hue crosses 0/180 (i.e. 179 is red and 1 is also red),
                // need to adjust to 90-270 with red at 180.
                // Otherwise, the average and standard deviation values are
                // invalid - average of 179 and 1 is 90 instead of 0.
                Mat redMask = new Mat();
                Mat redHSV = input.clone();
          
                // for redMask, find all values between 0-89
                Core.inRange(input, new Scalar(0,0,0), new Scalar(89,255,255), redMask);
                // convert them to 180-270 (by adding 180)
                Core.add(input, new Scalar(180,0,0), redHSV, redMask);

                // channel 0 is hue, with red at 180 +/- 10

                // get average and standard deviation
                MatOfDouble mean = new MatOfDouble();
                MatOfDouble stdDev = new MatOfDouble();
                Core.meanStdDev(redHSV, mean, stdDev);

                int average_hue_red = (int) mean.get(0, 0)[0];
                int stdDev_hue_red = (int) stdDev.get(0, 0)[0];

                // channel 1 is saturation
                int average_sat_red = (int) Core.mean(redHSV).val[1];

                if (average_hue_red > 170 && average_hue_red < 190 && average_sat_red > MinSaturation && stdDev_hue_red <= MaxStdDev)
                {
                    colorData.color = Color_Enum.Color_Red;
                    deltaHue = 180 - average_hue_red;
                    deltaSat = 150 - average_sat_red;
                }
            }

            // compute score
            if (deltaSat < 0) {
                deltaSat = 0;
            }
          
            colorData.score = 100 - Math.sqrt(deltaHue * deltaHue + deltaSat * deltaSat);
        }
          
        return colorData;
    }

    //
    // This function fills a list with the information on each color
    // seen in the specified region.
    //
    EnumMap<Color_Enum, ColorData> searchForColors(Mat input, int region) {

        EnumMap<Color_Enum, ColorData> bestColorDataList = new EnumMap<>(Color_Enum.class);
        ColorData workingColorData;
        DebugData bestDebug;
        Mat regionMat;
            
        // clear the color data
        for (Color_Enum colorLoop : Color_Enum.values())
        {
            bestColorDataList.put(colorLoop, new ColorData());
            bestColorDataList.get(colorLoop).squareCount = 0;
        }

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

                    workingColorData = computeColorData(square);

                    // this code saves data to draw a square around the 5x5 grid with the line color
                    // set to the color found in the grid (if any). The line width is small if it's
                    // not a great score, and large if it's a good score.
                    // The squares can be seen if using Camera Stream on the driver station or
                    // have a monitor connected to the HDMI port on the control hub.
                    int width = 1;
                    if (workingColorData.score >= 98)
                    {
                        width = 2;
                    }

                    Color_Enum color = workingColorData.color;

                    if (workingColorData.color != Color_Enum.Color_None)
                    {
                        // increment the square count for use when determining
                        // region with the most of a given color.
                        bestColorDataList.get(color).squareCount++;

                        // the square drawing data is saved off in a list and drawn later                  
                        DebugData data = new DebugData(color, upperLeft, lowerRight, width);
                        debugList.add(data);

                        if (workingColorData.score > bestColorDataList.get(color).score)
                        {
                            bestColorDataList.get(color).color = workingColorData.color;
                            bestColorDataList.get(color).score = workingColorData.score;
                            bestColorDataList.get(color).x_location = col2;
                            bestColorDataList.get(color).y_location = row2;
                            bestDebug = data;
                        }
                    }
                }
            }
        }
            
        return bestColorDataList;
    }

    @Override
    public void init(int width, int height, CameraCalibration calibration)
    {
        // initialize static color list
        DebugData debugData = new DebugData ();
    }

    @Override
    public void onDrawFrame(Canvas canvas, int onscreenWidth, int onscreenHeight, float scaleBmpPxToCanvasPx, float scaleCanvasDensity, Object userContext)
    {
        // In the future, this could be used to draw the debug data that is currently drawn in processFrame
    }

    @Override
    public Mat processFrame(Mat input, long captureTimeNanos)
    {
        Mat hsvMat = new Mat();
        ColorData colorData = new ColorData();

        // clear out previous data
        telemetryData.clear();
        debugList.clear();

        // convert to HSV format
        hsvMat = inputToHSV(input);

        // for each region, compute the color data
        for (int region = 0; region < NumRegions; ++region) {

            // compute and save color data
            region_colorData.add(region, searchForColors(hsvMat, region));
        }

        /*
         * Draw a rectangle showing each region on the screen.
         * Simply a visual aid. Serves no functional purpose.
         */
        Scalar WHITE = new Scalar (255, 255, 255);
        DebugData debugData = new DebugData();
        
        for (int region = 0; region < NumRegions; ++region)
        {
            Color_Enum color = Color_Enum.Color_None;

            if (region_colorData.get(region).get(Color_Enum.Color_Green).score > 0) {
                color = Color_Enum.Color_Green;
            }
            else if (region_colorData.get(region).get(Color_Enum.Color_Yellow).color == Color_Enum.Color_Yellow) {
                color = Color_Enum.Color_Yellow;
            }
            else if (region_colorData.get(region).get(Color_Enum.Color_Red).score > 0) {
                color = Color_Enum.Color_Red;
            }
            else if (region_colorData.get(region).get(Color_Enum.Color_Blue).score > 0) {
                color = Color_Enum.Color_Blue;
            }

            telemetryData.add (new TelemetryData("Region " + region + " Color", color));

            Point regionBottomRight = new Point(RegionTopLeft[region].x + RegionWidth[region],
                                                RegionTopLeft[region].y + RegionHeight[region]);

            // if no color found, draw region outline in white
            if (color == Color_Enum.Color_None)
            {
                color = Color_Enum.Color_White;
            }
            debugList.add(new DebugData(color, RegionTopLeft[region], regionBottomRight, 2));
        }

        telemetryData.add (new TelemetryData("Red region", getRegion (Color_Enum.Color_Red)));
        telemetryData.add (new TelemetryData("Blue region", getRegion (Color_Enum.Color_Blue)));

        // save data for use by the opMode
        synchronizedColorData.setColorData (region_colorData);
        synchronizedTelemetryData.setList (telemetryData);

        // clear out for next time
        telemetryData.clear();

        // draw debug data (square outlines) on the screen
        Mat output = input;
        for (DebugData data : debugList)
        {
            Scalar color = getColorScalar (data.color);

            Imgproc.rectangle(output, data.upperLeft, data.lowerRight, color, data.width);
        }

        return output;
    }

}
