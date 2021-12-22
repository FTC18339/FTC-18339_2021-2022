package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp
public class FTC_18339_TankProtocol001 extends Main {
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

        double[] qs = math.IKArm(5 * Algorithms002.mmPerInch, 5 * Algorithms002.mmPerInch, Math.PI / 2);

        base_arm_joint.setPosition(math.Clamp(qs[0] / Math.PI, 0, 1));
        second_arm_joint.setPosition(math.Clamp(qs[1] / Math.PI, 0, 1));
        hand.setPosition(math.Clamp(qs[2] / ((7/4) * Math.PI), 0, 1));
        gripper1.setPosition(1);
        gripper2.setPosition(0);

        //initColorSensor();

        if (opModeIsActive()) {

            while (opModeIsActive()) {
                // Show motor info on android phone

                //Runtime Loop to update hardware with input, and automation
                SetMotorForces();
                SetSpinnerForces();
                SetCraneIKTarget();
                SetGripperForces();

                if(gamepad2.x) {
                    double[] qg = math.IKArm(groundPosition[0], groundPosition[1], Math.PI / 2);

                    base_arm_joint.setPosition(math.Clamp(qg[0] / Math.PI, 0, 1));
                    second_arm_joint.setPosition(math.Clamp(qg[1] / Math.PI, 0, 1));
                    hand.setPosition(math.Clamp(qg[2] / ((7/4) * Math.PI), 0, 1));

                    currentX = groundPosition[0];
                    currentY = groundPosition[1];
                    currentPhi = Math.PI / 2;
                }

                if(gamepad2.y) {
                    double[] qr = math.IKArm(5 * Algorithms002.mmPerInch, 5 * Algorithms002.mmPerInch, Math.PI / 2);

                    base_arm_joint.setPosition(math.Clamp(qr[0] / Math.PI, 0, 1));
                    second_arm_joint.setPosition(math.Clamp(qr[1] / Math.PI, 0, 1));
                    hand.setPosition(math.Clamp(qr[2] / ((7/4) * Math.PI), 0, 1));

                    currentX = 5 * Algorithms002.mmPerInch;
                    currentY = 5 * Algorithms002.mmPerInch;
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
            spinner.setPower(1);
        } else if(b) {
            spinner.setPower(-1);
        } else {
            spinner.setPower(0);
        }
    }

    float craneRotationSpeedMultiplier = 0.015f;
    float craneArmJointMultiplier = 1.5f;
    float craneArmOrientationMultiplier = 0.01f;
    double currentX = 5 * Algorithms002.mmPerInch;
    double currentY = 5 * Algorithms002.mmPerInch;
    double currentPhi = Math.PI / 2;
    boolean firstIK = false;
    public void SetCraneIKTarget()
    {
        double x = currentX;
        double y = currentY;
        double phi = currentPhi;

        double orientationChange = 0;

        if(gamepad2.left_bumper)
            orientationChange -= 1;

        if(gamepad2.right_bumper)
            orientationChange += 1;

        float ticks = MAX_NUM_TICKS_ROTATOR;
        //what swath of the circle can the rotator rotate to? Half of the actual angle so 45 degrees represents a quarter of the circle
        float angleOfRotation = 45;
        //Half of the total range, positive and negative angle of rotation
        float rangeOfTicks = (angleOfRotation/360) * ticks;
        double r = (gamepad2.right_trigger - gamepad2.left_trigger) * craneRotationSpeedMultiplier * ROTATOR_RPM * ticks;
        if(!(arm_rotator.getCurrentPosition() >= rangeOfTicks && r > 0) && !(arm_rotator.getCurrentPosition() <= -rangeOfTicks && r < 0)) {
            arm_rotator.setVelocity(r);
        }

        if((gamepad2.left_stick_y == 0 && gamepad2.right_stick_y == 0 && orientationChange == 0)) return;

        double xChange = -gamepad2.left_stick_y * craneArmJointMultiplier;
        double yChange = gamepad2.right_stick_y * craneArmJointMultiplier;
        double phiChange = orientationChange * craneArmOrientationMultiplier;

        double[] target = math.IKTargetClamp(x + xChange, y + yChange);
        phi += phiChange;

        double[] qs = math.IKArm(target[0], target[1], phi);

        base_arm_joint.setPosition(math.Clamp(qs[0] / Math.PI, 0, 1));
        second_arm_joint.setPosition(math.Clamp(qs[1] / Math.PI, 0, 1));
        hand.setPosition(math.Clamp(qs[2] / ((7/4) * Math.PI), 0, 1));

        currentX = target[0];
        currentY = target[1];
        currentPhi = phi;
    }

    public void SetGripperForces() {
        if(gamepad2.a) {
            gripper1.setPosition(1);
            gripper2.setPosition(0);
        }

        if(gamepad2.b) {
            gripper1.setPosition(0);
            gripper2.setPosition(1);
        }
    }

    /*public void SetCraneForces()
    {
        //Rotation of the entire arm itself, right trigger is right rotation, left trigger is left
        //double r = gamepad2.right_trigger - gamepad2.left_trigger;
        double dPadUp = 0;
        double dPadDown = 0;
        if(gamepad2.dpad_up) {
            dPadUp = 1;
        }
        if(gamepad2.dpad_down) {
            dPadDown = 1;
        }
        //number
        float ticks = MAX_NUM_TICKS_ROTATOR;
        //what swath of the circle can the rotator rotate to? Half of the actual angle so 45 degrees represents a quarter of the circle
        float angleOfRotation = 45;
        //Half of the total range, positive and negative angle of rotation
        float rangeOfTicks = (angleOfRotation/360) * ticks;


        double r = (dPadUp - dPadDown) * craneRotationSpeedMultiplier * ROTATOR_RPM * ticks;
        //Rotation of the first arm joint at the base of the arm.
        double r1 = gamepad2.left_stick_y * craneArmJointOneMultiplier;
        //Rotation of the second arm joint.
        double r2 = gamepad2.right_stick_y * craneArmJointTwoMultiplier;

        if(!(arm_rotator.getCurrentPosition() >= rangeOfTicks && r > 0) && !(arm_rotator.getCurrentPosition() <= -rangeOfTicks && r < 0)) {
            arm_rotator.setVelocity(r);
        }
        /*base_arm_joint.setPosition(base_arm_joint.getPosition() + r1);
        second_arm_joint.setPosition(second_arm_joint.getPosition() + r2);
    }*/
}