package org.firstinspires.ftc.teamcode.util;

import com.acmerobotics.dashboard.config.Config;

    public final class Constants{
        @Config
        public static final class DriveConstants{

        }
        @Config
        public static final class outtakeSlideConstants{
            public static int HighBucket = 3100;//
            public static int passThroughScoringBelowChamber = 1685;
            public static int passThroughScoringOnChamber = 2552;
            public static int transfer = 0;
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

            public static double open = 0;
            public static double closed = 0;
            public static double wayOpen = 0;
            public static double gimbalReset = 0;

        }
        @Config
        public static final class v4bConstants{
            public static double ground = 0;
            public static double hover = 0;
            public static double tray = 0;
            public static double farInTrayForPassThrough = 0;
        }@Config

        public static final class outtakeAxleConstants{
            public static double passThrough = .6799;
            public static double specScoring = .1961;
            public static double HBScoring =.3949;
            public static double straightUp = .5;
            public static double down = .9773;



        }
        @Config
        public static final class outtakeClawConstants{
            public static double open = .8656;
            public static double closed = .573;

        }
        @Config
        public static final class trayConstants{
            public static double open = 0;
            public static double halfClosed = 0;
            public static double closed = 0;

        }


    }

