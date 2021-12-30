package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp
public class FTC18339TankProtocol003 extends Main {
    private double left_front_power = 0;
    private double left_back_power = 0;
    private double right_front_power = 0;
    private double right_back_power = 0;

    private double moveMultiplier = 1f;

    int dir = 1;
    double adder = 0;

    public double[] currentCraneCoordinates = new double[] {0,0};

    public double[] groundPosition = new double[] {207.24, 164.24};

    @Override
    public void runOpMode() {

        //Initialize all hardware, servos, and sensors, as well as the algorithm
        initMaths();
        initHardware();
        initManualModes();

        waitForStart();

        double[] qs = math.IKArm(8 * Algorithms002.mmPerInch, 8 * Algorithms002.mmPerInch, Math.PI / 2);

        base_arm_joint.setPosition(math.Clamp(1 - (qs[0] / Math.PI), 0, 1));
        second_arm_joint.setPosition(math.Clamp(1 - (qs[1] / Math.PI), 0, 1));
        hand.setPosition(math.Clamp(1 - (qs[2] / Math.PI), 0, 1));
        //gripper1.setPosition(1);

        //initColorSensor();

        if (opModeIsActive()) {

            while (opModeIsActive()) {
                // Show motor info on android phone

                //Runtime Loop to update hardware with input, and automation
                SetMotorForces();
                SetSpinnerForces();
                SetCraneIKTarget();
                SetGripperForces();

                telemetry.addData("q1; q2; q3", Math.toDegrees(math.tQ1) + "; " + Math.toDegrees(math.tQ2) + "; " + Math.toDegrees(math.tQ3));

                if(gamepad2.x) {
                    double[] qg = math.IKArm(groundPosition[0], groundPosition[1], Math.PI / 2);

                    base_arm_joint.setPosition(math.Clamp(1 - (qg[0] / Math.PI), 0, 1));
                    second_arm_joint.setPosition(math.Clamp(1 - (qg[1] / Math.PI), 0, 1));
                    hand.setPosition(math.Clamp(1 - (qg[2] / Math.PI), 0, 1));

                    currentX = groundPosition[0];
                    currentY = groundPosition[1];
                    currentPhi = Math.PI / 2;
                }

                if(gamepad2.y) {
                    double[] qr = math.IKArm(8 * Algorithms002.mmPerInch, 8 * Algorithms002.mmPerInch, Math.PI / 2);

                    base_arm_joint.setPosition(math.Clamp(1 - (qr[0] / Math.PI), 0, 1));
                    second_arm_joint.setPosition(math.Clamp(1 - (qr[1] / Math.PI), 0, 1));
                    hand.setPosition(math.Clamp(1 - (qr[2] / Math.PI), 0, 1));

                    currentX = 8 * Algorithms002.mmPerInch;
                    currentY = 8 * Algorithms002.mmPerInch;
                    currentPhi = Math.PI / 2;
                }

                //Update phone info
                telemetry.update();
                idle();
            }
        }
    }

    boolean wasMovingLastFrame = false;
    boolean notRunningConveyor = true;
    boolean notRunningRoller = true;
    public void SetMotorForces() {
        if(!NoNullHardware()) return;

        double time = System.currentTimeMillis();

        //Algorithm determined wheel forces with the inputs
        double y = gamepad1.left_stick_y;

        //Right trigger right rotation
        //Right trigger is right rotation, left trigger is left. Can slow rotations by depressing trigger less.
        float rAxis = gamepad1.right_trigger - gamepad1.left_trigger;

        left_front_power = -math.GetWheelForceTank(y, 1, rAxis);
        left_back_power = math.GetWheelForceTank(y, 2, rAxis);
        right_front_power = -math.GetWheelForceTank(y, 3, rAxis);
        right_back_power = math.GetWheelForceTank(y, 4, rAxis);

        telemetry.addData("lpb", left_back_power);
        telemetry.addData("lpf", left_front_power);
        telemetry.addData("rpb", right_back_power);
        telemetry.addData("rpf", right_front_power);

        left_front.setVelocity(left_front_power * MAX_NUM_TICKS_MOVEMENT * MOVEMENT_RPM);
        left_back.setVelocity(left_back_power * MAX_NUM_TICKS_MOVEMENT * MOVEMENT_RPM);
        right_front.setVelocity(right_front_power * MAX_NUM_TICKS_MOVEMENT * MOVEMENT_RPM);
        right_back.setVelocity(right_back_power * MAX_NUM_TICKS_MOVEMENT * MOVEMENT_RPM);
    }

    public void SetSpinnerForces() {
        if(!NoNullSpinner()) return;

        boolean x = gamepad1.x;
        boolean b = gamepad1.b;

        if(x) {
            telemetry.addData("x","x is active");
            spinner.setPower(-1);
        } else if(b) {
            spinner.setPower(1);
        } else {
            spinner.setPower(0);
        }
    }

    float craneRotationSpeedMultiplier = 0.01f;
    float craneArmJointMultiplier = 1.5f;
    float craneArmOrientationMultiplier = 0.01f;
    double currentX = 8 * Algorithms002.mmPerInch;
    double currentY = 8 * Algorithms002.mmPerInch;
    double currentPhi = Math.PI / 2;
    boolean firstIK = false;
    boolean lastTimeLeftStickInput = false;
    boolean lastTimeRightStickInput = false;

    double craneStepMM = 25.4;
    public void SetCraneIKTarget()
    {
        //placeholder variables for target
        double x = currentX;
        double y = currentY;
        double phi = currentPhi;

        double xChange = 0, yChange = 0, phiChange = 0;

        double orientationChange = 0;

        telemetry.addData("gamepad2left; gamepad2right", gamepad2.left_stick_y + "; " + gamepad2.right_stick_y);

        if(gamepad2.left_bumper)
            orientationChange = -1;
        else if(gamepad2.right_bumper)
            orientationChange = 1;

        /*if(gamepad2.left_stick_y != 0) {
            if(!lastTimeLeftStickInput) {
                xChange = craneStepMM * math.TrueSign(gamepad2.left_stick_y);
                lastTimeLeftStickInput = true;
            }
        } else {
            lastTimeLeftStickInput = false;
        }

        if(gamepad2.right_stick_y != 0) {
            if(!lastTimeRightStickInput) {
                yChange = craneStepMM * math.TrueSign(gamepad2.right_stick_y);
                lastTimeRightStickInput = true;
            }
        } else {
            lastTimeRightStickInput = false;
        }*/

        float ticks = MAX_NUM_TICKS_ROTATOR;
        //what swath of the circle can the rotator rotate to? Half of the actual angle so 45 degrees represents a quarter of the circle
        float angleOfRotation = 45;
        //Half of the total range, positive and negative angle of rotation
        float rangeOfTicks = (angleOfRotation/360) * ticks;
        //Set the velocity
        double r = (gamepad2.right_trigger - gamepad2.left_trigger) * craneRotationSpeedMultiplier * ROTATOR_RPM * ticks;
        if(!(arm_rotator.getCurrentPosition() >= rangeOfTicks && r > 0) && !(arm_rotator.getCurrentPosition() <= -rangeOfTicks && r < 0)) {
            arm_rotator.setVelocity(r);
        }

        if((gamepad2.left_stick_y == 0 && gamepad2.right_stick_y == 0 && orientationChange == 0)) return;

        //Get the inputs for the arm
        xChange = -gamepad2.left_stick_y * craneArmJointMultiplier;
        yChange = -gamepad2.right_stick_y * craneArmJointMultiplier;
        phiChange = orientationChange * craneArmOrientationMultiplier;

        //Clamp the controls
        double[] target = math.IKTargetClamp(x + xChange, y + yChange);
        phi += phiChange;

        //Calculate the angles the arm must take
        double[] qs = math.IKArm(target[0], target[1], phi);

        telemetry.addData("fQ1; fQ2; fQ3", Math.toDegrees(qs[0]) + "; " + Math.toDegrees(qs[1]) + "; " + Math.toDegrees(qs[2]));

        //Set the Servo Positions
        base_arm_joint.setPosition(math.Clamp(1 - (qs[0] / Math.PI), 0, 1));
        second_arm_joint.setPosition(math.Clamp(1 - (qs[1] / Math.PI), 0, 1));
        hand.setPosition(math.Clamp(1 - (qs[2] / Math.PI), 0, 1));

        currentX = target[0];
        currentY = target[1];
        currentPhi = phi;
    }

    public void SetGripperForces() {
        if(gamepad2.a) {
            gripper1.setPosition(1);
        }

        if(gamepad2.b) {
            gripper1.setPosition(0);
        }
    }
}
