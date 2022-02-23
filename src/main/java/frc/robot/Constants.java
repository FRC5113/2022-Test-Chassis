/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.util.Units;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants.  This class should not be used for any other purpose.  All constants should be
 * declared globally (i.e. public static).  Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {

	public static final class ChassisConstants {

		
		//CHASSIS MOTOR PORTS
		public static final int kFLChassis = 12;
		public static final int kFRChassis = 22;
		public static final int kBLChassis = 11;
		public static final int kBRChassis = 21;

		//CHARACTERIZATION VALUES FOR CHASSIS
		public static final double kTrackWidth = 0.4464852526536931;
		public static final double kSChassis = 0.0741;
		public static final double kVChassis = 2.87;
		public static final double kAChassis = 0.289;
		public static final double kPDriveVel = 0.05;
		public static final double gearBoxRatio = 8.1;

		//AUTON HELPERS
		public static final DifferentialDriveKinematics kDriveKinematics = new DifferentialDriveKinematics(kTrackWidth);
		public static final double kRamseteB = 2;
		public static final double kRamseteZ = 0.7;
	}

	public static final class DriveConstants {
		// These ones are currently in use by the Auton Code
		// These are example values only - DO NOT USE THESE FOR YOUR OWN ROBOT!
		// These characterization values MUST be determined either experimentally or
		// theoretically
		// for *your* robot's drive.
		// The Robot Characterization Toolsuite provides a convenient tool for obtaining
		// these
		// values for your robot.
  
  
		public static final double ksVolts = 0.50953;//0.53467;//0.50687; //0.52546;//0.50207
		public static final double kvVoltSecondsPerMeter = 2.6785;//2.6842;//2.6901;//0.0097548;//0.0097242
		public static final double kaVoltSecondsSquaredPerMeter = 0.12073;//0.13339;// 0.12386; //0.00040311;//0.00038467
  
		// Example value only - as above, this must be tuned for your drive!
		public static final double kPDriveVel =0.19624;//2.3549;//0.19624;//0.2089;//0.19968;//1.52739;//0.19968;//2.8551//xdxdc1.1847E-07;//0.014779 - currently set to CANcode Kpvalue
		public static final double kTrackwidthMeters = 0.45132;
		// public static final DifferentialDriveKinematics kDriveKinematics = ;
		public static final double kMaxSpeedMetersPerSecond = 2; 
		public static final double kMaxAccelerationMetersPerSecondSquared = 2;
		// Reasonable baseline values for a RAMSETE follower in units of meters and
		// seconds
		public static final double kRamseteB = 2;
		public static final double kRamseteZeta = 0.7;
	  }

	public static final class IntakeConstants {

		//INTAKE PISTON AND MOTOR *********** PORTS NEED UPDATING
		public static final int kIntakeMotor = 40;
		public static final int kLIntakePiston = 0;
		public static final int kRIntakePiston = 0;

		//public static final double kIntakeSpeed = 0.75;

	}

	public static final class IndexerConstants {

		//INDEXER MOTORS NEED TO BE ADDED

		public static final int kExtIndexMotor = 41;
		public static final int kMidIndexMotor = 42;
	}

	public static final class HopperConstants {

		//HOPPER MOTORS
		public static final int kHopperMotor = 43;

	}
	
	public static final class TurretConstants {

		//TURRET MOTOR PORT
		public static final int kTurretMotor = 44; 
		
		
			}

	public static final class LimelightConstants {

		//LIMELIGHT DISTANCE CALCULATION *********** ALL NEED UPDATING
		public static final double groundToLimelight = Units.inchesToMeters(24);
		public static final double groundToTarget = Units.inchesToMeters(50);
		public static final double limelightAngleDegress = 25;

	}
	
	public static final class ShooterConstants {

		//ALL NEED UPDATING

		//SHOOTER MOTOR PORTS
		public static final int kLShooterMotor = 31;
		public static final int kRShooterMotor = 32;
		
		//HOOD PISTON PORTS
		public static final int kRHoodPiston = 0;
		
		public static final int kLHoodPiston = 0;

		//S AND V VALUES FOR SHOOTER
		public static final double kSShooter = 0;
		public static final double kVShooter = 0;

		//PID VALUES FOR SHOOTER
		public static final double kShooterkP = 0;
		public static final double kShooterkI = 0;
		public static final double kShooterkD = 0;

	}

	public static final class ClimberConstants {

		//CLIMBER MOTOR AND SPEED *********** NEED TO BE UPDATED
		public static final int kClimber = 0;
		public static final double kClimbSpeed = 0.25;

	}

	public static final class LimeConstants {
		//limelight constants
		public static final double ANGLE = 51.09; //43.09
		public static final double targetHeight = 69; //49.625 mediacenter table //48.75 + 27.75; // 27.75 is the trashcan
		public static final double limelightHeight = 5.5;
        public static final double kP = 0.025; //0.015 works in cafeteria
        public static final double kI = 0.00025; //0.00015 works in cafeteria
        public static final double kD = 0.0005; //0.015 works in cafeteria
		public static final double inchesToMeters = 0.0254;
	}
		

	
	
	
	


}
