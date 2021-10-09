package org.firstinspires.ftc.teamcode;

import android.app.Activity;
import android.graphics.Color;
import android.view.View;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.vuforia.HINT;
import com.vuforia.Vuforia;

import java.util.ArrayList;
import java.util.List;

public class Main extends LinearOpMode {

    public DcMotorEx right_back;
    public DcMotorEx right_front;
    public DcMotorEx left_back;
    public DcMotorEx left_front;
    public DcMotor roller;
    public DcMotorEx shooter;
    public DcMotorEx arm;
    public DcMotor conveyor;

    public Servo hand;
    public Servo hand2;

    public ColorSensor colorSensor;
    float hsvValues[] = {0F, 0F, 0F};
    final float values[] = hsvValues;
    final double SCALE_FACTOR = 255;

    public TouchSensor touch;

    private static final String TFOD_MODEL_ASSET = "UltimateGoal.tflite";
    private static final String LABEL_FIRST_ELEMENT = "Quad";
    private static final String LABEL_SECOND_ELEMENT = "Single";

    private static final String VUFORIA_KEY =
            "AVv4k/j/////AAABmVGjqmYKvECglYRfGNuLUhR6OKOwdDO0HslRqSgO4WqxXWxsQFMf6NIdGlqA4PIuYXpyNYc+zH2A33zuJIKFxIQiOP+3Mxd+QfIiTGHNDHolKhhbSLAzv4cMtMP/LaS2pODkGpR11mQLXfkNTinuK5P3dwQUuW46/lduuGU+Ue0QhKV23i41YTOUl3sLs4UTtOG22Q9fRtYVWILVWwRDa5M/PWU87MYK1Zh26I2/KUj1kn5OKedND0JPtu6+vGU9tyDeuqLaJa6aWnZXjbXcWLXPgmTBWZWHIAh/5QvdGB6C34lYDtKOmtSB5pwQTz93cAuqyXhoEx6Vy0lBz1EynoaBjntRBirjnc9TapAOMb3K";

    public VuforiaTrackables visionTargets;
    public VuforiaLocalizer vuforiaLocalizer;
    private VuforiaLocalizer.Parameters parameters;
    public VuforiaTrackable blueTowerTarget;
    public VuforiaTrackable redTowerTarget;
    public VuforiaTrackable redAlliance;
    public VuforiaTrackable blueAlliance;
    public VuforiaTrackable frontWall;

    public OpenGLMatrix lastKnownLocation;
    public OpenGLMatrix phoneLocation;
    public OpenGLMatrix lastKnownPose;

    public TFObjectDetector tfod;

    public Algorithms math;

    public List<VuforiaTrackable> allTrackables;

    public final float MAX_NUM_TICKS_MOVEMENT = 537.6f;
    public final float MAX_NUM_TICKS_ARM = 5264f;
    public final float MAX_NUM_TICKS_SHOOTER = 28f;

    public final int MOVEMENT_RPM = 25;
    public final int ARM_RPM = 30;
    public final int SHOOTER_RPM = 6000;

    @Override
    public void runOpMode() { }

    public void initMaths() {

        //Initialize the maths program
        math = new Algorithms();
    }

    public void initVuforia() {
        float mmPerInch = Algorithms.mmPerInch;

        //Setup Vuforia Settings
        int cameraMonitorId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        parameters = new VuforiaLocalizer.Parameters(cameraMonitorId);
        parameters.cameraName = hardwareMap.get(WebcamName.class, "Webcam 1");
        parameters.vuforiaLicenseKey = VUFORIA_KEY;
        parameters.cameraDirection = VuforiaLocalizer.CameraDirection.BACK;
        parameters.useExtendedTracking = false;

        //Create vuforia instance with settings
        vuforiaLocalizer = ClassFactory.getInstance().createVuforia(parameters);
        Vuforia.setHint(HINT.HINT_MAX_SIMULTANEOUS_IMAGE_TARGETS, 2);

        //Set targets (images)
        visionTargets = vuforiaLocalizer.loadTrackablesFromAsset("UltimateGoal");

        //initVuforiaTargets();

        //Set phone location on the bot (camera)
        //if(math != null)
        //phoneLocation = math.createMatrix(0, 0, 0, 0, 0, 0);

        long time = System.currentTimeMillis() + 10;
        while(System.currentTimeMillis() < time && opModeIsActive()) {
            float x = 0;
            x++;
        }

        float CAMERA_VERTICAL_DISPLACEMENT = 4 * mmPerInch;
        float CAMERA_LEFT_DISPLACEMENT = -6 * mmPerInch;
        float CAMERA_FORWARD_DISPLACEMENT = 9 * mmPerInch;

        //Setup listener to determine position and rotation
        /*if(allTrackables != null && math != null) {
            for(VuforiaTrackable target : allTrackables) {
                ((VuforiaTrackableDefaultListener) target.getListener()).setPhoneInformation(
                        math.createMatrix(CAMERA_FORWARD_DISPLACEMENT, CAMERA_LEFT_DISPLACEMENT, CAMERA_VERTICAL_DISPLACEMENT, -90, 0, 0),
                        parameters.cameraDirection);
            }
        }*/
    }

    public void initVuforiaTargets() {
        float halfField = Algorithms.halfField;
        float quadField = Algorithms.quadField;
        float mmHeight = Algorithms.mmTargetHeight;

        if(visionTargets != null) {
            blueTowerTarget = visionTargets.get(0);
            redTowerTarget = visionTargets.get(1);
            redAlliance = visionTargets.get(2);
            blueAlliance = visionTargets.get(3);
            frontWall = visionTargets.get(4);

            blueTowerTarget.setName("Blue Tower Target");
            redTowerTarget.setName("Red Tower Target");
            redAlliance.setName("Red Alliance");
            blueAlliance.setName("Blue Alliance");
            frontWall.setName("Front Wall");

            allTrackables = new ArrayList<>();
            allTrackables.addAll(visionTargets);

            /*if(math != null) {
                blueTowerTarget.setLocation(math.createMatrix(-quadField, halfField, mmHeight, 90, 0, 180));
                redTowerTarget.setLocation(math.createMatrix(quadField, halfField, mmHeight, 90, 0, 180));
                redAlliance.setLocation(math.createMatrix(halfField, 0, mmHeight, 90, 0, -90));
                blueAlliance.setLocation(math.createMatrix(-halfField, 0, mmHeight, 90, 0, 90));
                frontWall.setLocation(math.createMatrix(0, -halfField, mmHeight, 90, 0, 0));
            }*/
        }
    }

    public void initTfod() {
        //set the Tensor Flow reference

        int tfodMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
                "tfodMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);

        //How confident is Tensor Flow on it's detections? Lowered from 0.8 so Tensor Flow detects the rings easier.
        tfodParameters.minResultConfidence = 0.6f;

        //Use Vuforia to initialize Tensor Flow
        tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforiaLocalizer);
        tfod.loadModelFromAsset(TFOD_MODEL_ASSET, LABEL_FIRST_ELEMENT, LABEL_SECOND_ELEMENT);
    }

    public void initHardware() {
        //Get references for all motor variables

        right_back = hardwareMap.get(DcMotorEx.class, "right_back");
        right_front = hardwareMap.get(DcMotorEx.class, "right_front");
        left_back = hardwareMap.get(DcMotorEx.class, "left_back");
        left_front = hardwareMap.get(DcMotorEx.class, "left_front");
        //arm = hardwareMap.get(DcMotorEx.class, "arm");
        //roller = hardwareMap.get(DcMotor.class, "roller");
        //shooter = hardwareMap.get(DcMotorEx.class, "shooter");
        //hand = hardwareMap.get(Servo.class, "hand");
        //hand2 = hardwareMap.get(Servo.class, "hand2");
        //conveyor = hardwareMap.get(DcMotor.class, "conveyor");
        //touch = hardwareMap.get(TouchSensor.class, "touch_sensor");
        //colorSensor = hardwareMap.get(ColorSensor.class, "color_sensor");

        // Set the functions of the motors.
        SetAutonomousDirection();
        right_back.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        right_front.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        left_back.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        left_front.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        //arm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        //roller.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        //shooter.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        //conveyor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
    }

    //Control Direction of wheels
    public void SetAutonomousDirection() {
        right_back.setDirection(DcMotorSimple.Direction.FORWARD);
        right_front.setDirection(DcMotorSimple.Direction.REVERSE);
        left_back.setDirection(DcMotorSimple.Direction.FORWARD);
        left_front.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    public void initManualModes() {
        resetMotorsAutonomous();
        //arm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        right_back.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        right_front.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        left_back.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        left_front.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        //shooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        //conveyor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        //roller.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public void initAutonomousModes() {
        resetMotorsAutonomous();
        //arm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        right_back.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        right_front.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        left_back.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        left_front.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        //shooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        //conveyor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public void StopMotors() {
        right_back.setVelocity(0);
        left_back.setVelocity(0);
        right_front.setVelocity(0);
        left_front.setVelocity(0);
    }

    public void resetMotorsAutonomous() {
        //arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        right_back.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        right_front.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        left_back.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        left_front.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        //shooter.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    public void RunToPositionArmAutonomous() {
        arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    public void RunToPositionAutonomousMovement() {
        right_back.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        right_front.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        left_back.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        left_front.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    public void RunToPositionAutonomousMovementROT() {
        right_back.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        right_front.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        left_back.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        left_front.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void initColorSensor() {
        //Get details to set the color fo the background. Makes it easy to distinguish ring.
        int relativeLayoutId = hardwareMap.appContext.getResources().getIdentifier("RelativeLayout", "id", hardwareMap.appContext.getPackageName());
        final View relativeLayout = ((Activity) hardwareMap.appContext).findViewById(relativeLayoutId);

        CalibrateColorSensor(relativeLayout);
    }

    private void CalibrateColorSensor(final View relativeLayout) {
        if(!NoNullSensors()) return;

        //Convert to HUE_SATURATION_VALUES from RED_BLUE_GREEN
        Color.RGBToHSV((int) (colorSensor.red() * SCALE_FACTOR),
                (int) (colorSensor.green() * SCALE_FACTOR),
                (int) (colorSensor.blue() * SCALE_FACTOR),
                hsvValues);

        //Set the background
        relativeLayout.post(new Runnable() {
            public void run() {
                relativeLayout.setBackgroundColor(Color.HSVToColor(0xff, values));
            }
        });
    }

    //Checks if these references are active and not null (empty).

    public boolean NoNullHardware() { return (left_back != null && left_front != null && right_back != null && right_front != null); }

    public boolean NoNullIntake() { return (roller != null && conveyor != null); }

    public boolean NoNullSensors() { return (colorSensor != null && touch != null); }

    public boolean NoNullArmature() { return (arm != null && hand != null); }

    public boolean NoNullShooter() { return (shooter != null); }
}
