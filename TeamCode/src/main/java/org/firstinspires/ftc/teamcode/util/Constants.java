package org.firstinspires.ftc.teamcode.util;

import com.acmerobotics.dashboard.config.Config;

    public final class Constants{
        @Config
        public static final class DriveConstants{

        }
        @Config
        public static final class outtakeSlideConstants{
            public static int HighBucket = 3100;//
            public static int passThroughScoringBelowChamber = 1864;
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

            public static int MAX = 1661;
            public static double power = .9;

        }
        @Config
        public static final class intakeClawConstants{

            //open: .4887
            //closed: .2951
            //gimbal reset: .4825
            public static double open = .4887;
            public static double closed = .2951;
            //public static double wayOpen = 0;
            public static double gimbalReset = .4825;

        }
        @Config
        public static final class v4bConstants{
            public static double ground = 1;
            public static double hover = 0;
            public static double tray = .149;
            public static double farInTrayForPassThrough = 0;
        }@Config

        public static final class outtakeAxleConstants{
            public static double passThrough = .6954;
            public static double specScoring = .1602;
            public static double HBScoring =.3949;
            public static double straightUp = .5;
            public static double down = .9439;



        }
        @Config
        public static final class outtakeClawConstants{
            public static double open = .8656;
            public static double closed = .573;

        }
        @Config
        public static final class trayConstants{
            public static double open = .8854;
            public static double halfClosed = 0;
            public static double closed = 0.4862;

        }


    }

