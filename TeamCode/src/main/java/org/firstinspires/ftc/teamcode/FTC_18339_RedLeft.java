package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous
public class FTC_18339_RedLeft extends FTC_18339_AutonomousProtocol004 {
    @Override
    public void ChildCommandInitialization(){
        commands = new Command[] {
                new Command("IKTOPDROP", 0, true, 0),
                new Command("MOVE", -firstMoveC, true, 0),
                new Command("ROTATE", Math.toRadians(rotationC - 5), true, 0),
                new Command("MOVE", -secondMoveC, true, 0),
                new Command("GRIPPERDROP", 0, true, 0),
                new Command("MOVE", thirdMoveC-1, true, 0),
                new Command("REDSPINNER", 0, true, 0),
                new Command("ROTATE", Math.toRadians(-rotationC - 10), true, 0),
                new Command("MOVE", -fourthMoveC, true, 0),
                new Command("IKFORSTART", 0, true, 0)
        };
    }
}
