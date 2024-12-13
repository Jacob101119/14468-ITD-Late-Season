package org.firstinspires.ftc.teamcode.util;

import com.acmerobotics.dashboard.config.Config;

    public final class Constants{
        @Config
        public static final class DriveConstants{

        }
        @Config
        public static final class outtakeSlideConstants{
            public static int HighBucket = 3100;//
            public static int passThroughScoringBelowChamber = 1780;
            public static int passThroughScoringOnChamber = 2552;
            public static int transfer = 910;
            public static int onChamber = 890;
            public static int aboveChamber = 1685;
            public static int prepForHang = 0;
            public static int MAX = 3100;
            public static double power = .9;


        }
        @Config
        public static final class intakeSlideConstants{

            public static int MAX = 2000;
            public static double power = .4;
            public static int transfer = 1000;
            public static int transfer2 = 750;
            public static int minFromGround = 718;//change

        }
        @Config
        public static final class intakeClawConstants{

            //open: .4887
            //closed: .2951
            //gimbal reset: .4825
            public static double gap45 = -.16765;//change
            public static double open = .4887;
            public static double closed = .2951;
            public static double looseGrab = .3515;
            //public static double wayOpen = 0;
            public static double gimbalReset = .4825;

            public static double turn90 = .808;

            //90 .132
            //.8026
           // 4 increments


        }
        @Config
        public static final class v4bConstants{
            public static double ground = 0.055;
            public static double hover = .15;
            public static double up = .805;
            public static double half = .4519;
            public static double transfer = .8499;
            //public static double farInTrayForPassThrough = .7781;
        }
        @Config
        public static final class outtakeAxleConstants{
            public static double passThrough = .7774;
            public static double specScoring = .2677;
            public static double HBScoring =.3949;
            public static double straightUp = .58;
            public static double down = .9439;
            public static double transfer = .7298;
            public static double autoStart = .6482;



        }
        @Config
        public static final class outtakeClawConstants{
            public static double open = .8656;
            public static double closed = .59;

        }
        @Config
        public static final class trayConstants{
            public static double open = .8854;
            public static double halfClosed = 0;
            public static double closed = 0.4862;

        }


    }

