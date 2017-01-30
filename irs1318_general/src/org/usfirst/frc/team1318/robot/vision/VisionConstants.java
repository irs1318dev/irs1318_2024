package org.usfirst.frc.team1318.robot.vision;

import org.opencv.core.Scalar;

public class VisionConstants
{
    // Debug output settings:
    public static final boolean DEBUG = true;
    public static final boolean DEBUG_PRINT_OUTPUT = true;
    public static final boolean DEBUG_PRINT_ANALYZER_DATA = false;
    public static final int DEBUG_FPS_AVERAGING_INTERVAL = 30;
    public static final boolean DEBUG_FRAME_OUTPUT = false;
    public static final int DEBUG_FRAME_OUTPUT_GAP = 30; // the number of frames to wait between saving debug image output
    public static final String DEBUG_OUTPUT_FOLDER = "/C/vision/";

    // Settings for AXIS IP-based camera
    public static final String AXIS_CAMERA_IP_ADDRESS = "10.13.18.11";
    public static final String AXIS_CAMERA_USERNAME_PASSWORD = "root:1318";
    public static final int AXIS_CAMERA_RESOLUTION_X = 320;
    public static final int AXIS_CAMERA_RESOLUTION_Y = 240;
    public static final double AXIS_CAMERA_ANGLE_OF_VIEW = 50.0; // note that documentation says 47 degrees, so we'll have to see whether this is accurate enough.
    public static final int AXIS_CAMERA_CENTER_WIDTH = VisionConstants.AXIS_CAMERA_RESOLUTION_X / 2; // distance from center to left/right sides in pixels
    public static final int AXIS_CAMERA_CENTER_HEIGHT = VisionConstants.AXIS_CAMERA_RESOLUTION_Y / 2; // distance from center to top/bottom in pixels
    public static final double AXIS_CAMERA_CENTER_VIEW_ANGLE = VisionConstants.AXIS_CAMERA_ANGLE_OF_VIEW / 2.0;

    // Settings for Microsoft LifeCam HD-3000 USB-based camera
    public static final int LIFECAM_CAMERA_RESOLUTION_X = 320;
    public static final int LIFECAM_CAMERA_RESOLUTION_Y = 240;
    public static final double LIFECAM_CAMERA_ANGLE_OF_VIEW = 45.0; // note that documentation says 68.5 degrees diagonal (at 16:9), so this is an estimate.
    public static final int LIFECAM_CAMERA_CENTER_WIDTH = VisionConstants.LIFECAM_CAMERA_RESOLUTION_X / 2; // distance from center to left/right sides in pixels
    public static final int LIFECAM_CAMERA_CENTER_HEIGHT = VisionConstants.LIFECAM_CAMERA_RESOLUTION_Y / 2; // distance from center to top/bottom in pixels
    public static final double LIFECAM_CAMERA_CENTER_VIEW_ANGLE = VisionConstants.LIFECAM_CAMERA_ANGLE_OF_VIEW / 2.0;
    public static final int LIFECAM_CAMERA_EXPOSURE = 30;
    public static final int LIFECAM_CAMERA_BRIGHTNESS = 30;
    public static final int LIFECAM_CAMERA_FPS = 25; // Max supported value is 30

    // Undistort constants
    public static final boolean SHOULD_UNDISTORT = false;

    // HSV Filtering constants
    public static final Scalar AXIS_HSV_FILTER_LOW = new Scalar(85, 65, 65);
    public static final Scalar AXIS_HSV_FILTER_HIGH = new Scalar(90, 255, 255);
    public static final Scalar LIFECAM_HSV_FILTER_LOW = new Scalar(75, 100, 100);
    public static final Scalar LIFECAM_HSV_FILTER_HIGH = new Scalar(85, 255, 255);

    // Contour filtering constants
    public static final double CONTOUR_MIN_AREA = 125.0;
}