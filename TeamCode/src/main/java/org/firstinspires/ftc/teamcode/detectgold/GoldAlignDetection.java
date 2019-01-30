package org.firstinspires.ftc.teamcode.detectgold;

import com.disnodeteam.dogecv.CameraViewDisplay;
import com.disnodeteam.dogecv.DogeCV;
import com.disnodeteam.dogecv.Dogeforia;
import com.disnodeteam.dogecv.detectors.roverruckus.GoldAlignDetector;
import com.disnodeteam.dogecv.filters.DogeCVColorFilter;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;

public class GoldAlignDetection {
    private Dogeforia vuforia;
    private GoldAlignDetector detector;

    public GoldAlignDetection(HardwareMap hardwareMap, String VUFORIA_KEY, VuforiaLocalizer.CameraDirection CAMERA_CHOICE, int cameraIndex) {
        detector = new GoldAlignDetector();
        detector.init(hardwareMap.appContext, CameraViewDisplay.getInstance(), cameraIndex, true);
        detector.useDefaults();

        // Optional Tuning
        detector.alignSize = 100; // How wide (in pixels) is the range in which the gold object will be aligned. (Represented by green bars in the preview)
        detector.alignPosOffset = 0; // How far from center frame to offset this alignment zone.
        detector.downscale = 0.4; // How much to downscale the input frames

        detector.areaScoringMethod = DogeCV.AreaScoringMethod.MAX_AREA; // Can also be PERFECT_AREA
        //detector.perfectAreaScorer.perfectArea = 10000; // if using PERFECT_AREA scoring
        detector.maxAreaScorer.weight = 0.005;

        detector.ratioScorer.weight = 5;
        detector.ratioScorer.perfectRatio = 1.0;

        Dogeforia.Parameters parameters = new Dogeforia.Parameters();
        parameters.vuforiaLicenseKey = "ATQZG9T/////AAABmVE44Js1eEitobSAT11TPWgVWpehtY2ffxf0OR4SebS7RHP+3yzoO+VrtWVCLxMYUmSBmrrU6wgXGe+ngM0D40IuvC2yHn4XxbrTTWY3l1/LU1XizPh5DnJ+m08z/VKW47kIC165vOgHef7HXSaJkWZNG0ovW3UTfTXOjA3YOvso2EPYP9gFmi9a2ak0VB6iqew9WpfVCKCX8ehTNa9duNSuCmodIqWTc+S90/VgPVo086NlcecvEadyJjz6U8YZK/o9VRPh2sQ9SlXApo3y2m9dZbiwBvfs5a0GVLBzdzwUh/0hLhUOZZndJY5+6CgGsL/0yEukEAFpMslapJGDARdSa7eRvYqnOCZsjMmxbRE6";

        parameters.cameraDirection = CAMERA_CHOICE;
        parameters.fillCameraMonitorViewParent = true;

        detector.enable();
        vuforia = new Dogeforia(parameters);
        vuforia.enableConvertFrameToBitmap();

        vuforia.setDogeCVDetector(detector);
        vuforia.enableDogeCV();
        vuforia.start();
    }

    /**
     * Returns if the gold element is aligned
     *
     * @return if the gold element is aligned
     */
    public boolean isAligned() {
        return detector.getAligned();
    }

    /**
     * Returns gold element last x-position
     *
     * @return last x-position in screen pixels of gold element
     */
    public double getGoldXPos() {
        return detector.getXPosition();
    }

    public void disable() {
        detector.disable();
    }

    public void stopVuforia() {
        vuforia.stop();
    }

    public Dogeforia getVuforia() {
        return vuforia;
    }
}
