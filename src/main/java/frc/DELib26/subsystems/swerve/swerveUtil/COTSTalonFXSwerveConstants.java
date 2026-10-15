package frc.DELib26.subsystems.swerve.swerveUtil;

import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Distance;

import static edu.wpi.first.math.util.Units.radiansPerSecondToRotationsPerMinute;

/* Contains values and required settings for common COTS swerve modules.
 * originally by FRC team 364 https://github.com/dirtbikerxz/BaseTalonFXSwerve/blob/main/src/main/java/frc/lib/util/COTSTalonFXSwerveConstants.java
 */
public class COTSTalonFXSwerveConstants {
	/**
	 * Diameter of the wheel in meters.
	 */
	private double wheelDiameter;

	public double wheelCircumference;
	/**
	 * Ratio between the drive motor shaft and the output shaft the wheel is mounted on.
	 */
	public final double driveGearRatio;
	/**
	 * Ratio between the steer motor shaft and the steer output shaft.
	 */
	public final double steerGearRatio;
	/**
	 * The coupled gear ratio between the CanCoder and the drive motor.
	 * Every 1 rotation of the steer motor results in coupled ratio of drive turns.
	 */
	public final double couplingGearRatio;

	public final DCMotor driveMotor;
	public final DCMotor steerMotor;

	public final boolean driveMotorInvert;
	public final boolean steerMotorInvert;
	public final boolean cancoderInvert;

	/**
	 * Wheel radius. Accuracy in these measurements affects wheel odometry
	 * which measures distance as a function of the number of rotations * wheel
	 * circumference.
	 */
	public void setWheelDiameter(Distance wheelDiameter) {
		this.wheelDiameter = wheelDiameter.in(Units.Meters);
		this.wheelCircumference = this.wheelDiameter * Math.PI;
	}

	public double getWheelDiameter() {
		return this.wheelDiameter;
	}

	public double getWheelRadius() {
		return this.wheelDiameter / 2.0;
	}

	public static InvertedValue isInvertedAsInvertedValue(boolean inverted) {
		return inverted ? InvertedValue.Clockwise_Positive : InvertedValue.CounterClockwise_Positive;
	}

	public static SensorDirectionValue isInvertedAsSensorDirectionValue(boolean inverted) {
		return inverted ? SensorDirectionValue.Clockwise_Positive : SensorDirectionValue.CounterClockwise_Positive;
	}

	/**
	 * Calculates the theoretical maximum linear speed of the robot in meters per second.
	 * Note: It is recommended to reduce this value by 10-15% to account for real-world inefficiencies.
	 * @return Theoretical maximum linear speed in meters per second.
	 */
	public double getTheoreticalMaxLinearSpeedMps() {
		return (radiansPerSecondToRotationsPerMinute(this.driveMotor.freeSpeedRadPerSec) / this.driveGearRatio)
				* (1 / 60.0)
				* this.wheelCircumference;
	}

	public COTSTalonFXSwerveConstants(
			Distance wheelDiameter,
			double steerGearRatio,
			double driveGearRatio,
			double couplingGearRatio,
			DCMotor driveMotor,
			DCMotor steerMotor,
			Boolean driveMotorInvert,
			Boolean steerMotorInvert,
			Boolean cancoderInvert
		) {
		this.setWheelDiameter(wheelDiameter);
		this.steerGearRatio = steerGearRatio;
		this.driveGearRatio = driveGearRatio;
		this.couplingGearRatio = couplingGearRatio;
		this.driveMotor = driveMotor;
		this.steerMotor = steerMotor;
		this.driveMotorInvert = driveMotorInvert;
		this.steerMotorInvert = steerMotorInvert;
		this.cancoderInvert = cancoderInvert;
	}

	/** West Coast Products */
	public static final class WCP {

		/** Common utility */
		private static final Distance WHEEL_DIAMETER = Units.Inches.of(4.0);

		private static final int FIRST_STAGE_DRIVEN_TEETH = 50;

		private static double calculateCouplingGearRatio(int pinionTeeth) {
			if (pinionTeeth < 1) throw new IllegalArgumentException("pinionTeeth must be >= 1");
			return (double) FIRST_STAGE_DRIVEN_TEETH / pinionTeeth;
		}

		/** West Coast Products - SwerveX Standard */
		public static final class SwerveXStandard {

			/** Steer (azimuth) gear reduction: motor revs per 1 azimuth rev */
			private static final double STEER_GEAR_RATIO = 396.0 / 35.0; // ≈ 11.314

			/** Falcon 500 */
			public static COTSTalonFXSwerveConstants Falcon500(DriveRatios driveRatio) {
				return create(driveRatio, DCMotor.getFalcon500(1), DCMotor.getFalcon500(1));
			}

			/** Kraken X60 */
			public static COTSTalonFXSwerveConstants KrakenX60(DriveRatios driveRatio) {
				return create(driveRatio, DCMotor.getKrakenX60(1), DCMotor.getKrakenX60(1));
			}

			private static COTSTalonFXSwerveConstants create(DriveRatios driveRatio, DCMotor driveMotor, DCMotor steerMotor) {

				boolean driveMotorInvert = false;
				boolean steerMotorInvert = true;
				boolean cancoderInvert = false;

				return new COTSTalonFXSwerveConstants(
						WHEEL_DIAMETER,
						STEER_GEAR_RATIO,
						driveRatio.overallGearRatio,
						calculateCouplingGearRatio(driveRatio.drivingGearTeeth),
						driveMotor,
						steerMotor,
						driveMotorInvert,
						steerMotorInvert,
						cancoderInvert);
			}

			public enum DriveRatios {
				X1_10(10, 7.85),
				X1_11(11, 7.13),
				X1_12(12, 6.54),
				X2_10(10, 6.56),
				X2_11(11, 5.96),
				X2_12(12, 5.46),
				X3_12(12, 5.14),
				X3_13(13, 4.75),
				X3_14(14, 4.41);

				public final int drivingGearTeeth;
				public final double overallGearRatio;

				DriveRatios(int drivingGearTeeth, double overallGearRatio) {
					this.drivingGearTeeth = drivingGearTeeth;
					this.overallGearRatio = overallGearRatio;
				}
			}
		}

		/** West Coast Products - SwerveX Flipped */
		public static final class SwerveXFlipped {

			/** Steer (azimuth) gear reduction: motor revs per 1 azimuth rev */
			private static final double STEER_GEAR_RATIO = 468.0 / 35.0; // ≈ 13.371

			/** Falcon 500 */
			public static COTSTalonFXSwerveConstants Falcon500(DriveRatios driveRatio) {
				return create(driveRatio, DCMotor.getFalcon500(1), DCMotor.getFalcon500(1));
			}

			/** Kraken X60 */
			public static COTSTalonFXSwerveConstants KrakenX60(DriveRatios driveRatio) {
				return create(driveRatio, DCMotor.getKrakenX60(1), DCMotor.getKrakenX60(1));
			}

			private static COTSTalonFXSwerveConstants create(DriveRatios driveRatio, DCMotor driveMotor, DCMotor steerMotor) {

				boolean driveMotorInvert = false;
				boolean steerMotorInvert = true;
				boolean cancoderInvert = false;

				return new COTSTalonFXSwerveConstants(
						WHEEL_DIAMETER,
						STEER_GEAR_RATIO,
						driveRatio.overallGearRatio,
						calculateCouplingGearRatio(driveRatio.drivingGearTeeth),
						driveMotor,
						steerMotor,
						driveMotorInvert,
						steerMotorInvert,
						cancoderInvert);
			}

			public enum DriveRatios {
				X1_10(10, 8.10),
				X1_11(11, 7.36),
				X1_12(12, 6.75),
				X2_10(10, 6.72),
				X2_11(11, 6.11),
				X2_12(12, 5.60),
				X3_10(10, 5.51),
				X3_11(11, 5.01),
				X3_12(12, 4.59);

				public final int drivingGearTeeth;
				public final double overallGearRatio;

				DriveRatios(int drivingGearTeeth, double overallGearRatio) {
					this.drivingGearTeeth = drivingGearTeeth;
					this.overallGearRatio = overallGearRatio;
				}
			}
		}
	}

	/** Swerve Drive Specialities */
	public static final class SDS {
		/** Swerve Drive Specialties - MK3 Module*/
		public static final class MK3 {

			private static final Distance WHEEL_DIAMETER = Units.Inches.of(4.0);

			/** Steer (azimuth) gear reduction for SDS MK3: motor revs per 1 azimuth rev */
			private static final double STEER_GEAR_RATIO = 12.8; // 12.8 : 1

			/** Teeth on the first-stage driven gear (SDS MK3/MK4/MK4i all use 50T) */
			private static final int FIRST_STAGE_DRIVEN_TEETH = 50;

			/** CTRE definition: drive motor revs per 1 azimuth rev = inverse of first drive stage */
			private static double calculateCouplingGearRatio(int pinionTeeth) {
				if (pinionTeeth < 1) throw new IllegalArgumentException("pinionTeeth must be >= 1");
				return (double) FIRST_STAGE_DRIVEN_TEETH / pinionTeeth;
			}

			private static COTSTalonFXSwerveConstants create(DriveRatios driveRatio, DCMotor driveMotor, DCMotor steerMotor) {

				boolean driveMotorInvert = false;
				boolean steerMotorInvert = false;
				boolean cancoderInvert = false;

				return new COTSTalonFXSwerveConstants(
						WHEEL_DIAMETER,
						STEER_GEAR_RATIO,
						driveRatio.overallGearRatio,
						calculateCouplingGearRatio(driveRatio.drivingGearTeeth),
						driveMotor,
						steerMotor,
						driveMotorInvert,
						steerMotorInvert,
						cancoderInvert);
			}

			/** Swerve Drive Specialties - MK3 Module (Falcon 500) */
			public static COTSTalonFXSwerveConstants Falcon500(DriveRatios driveRatio) {
				return create(driveRatio, DCMotor.getFalcon500(1), DCMotor.getFalcon500(1));
			}

			/** Swerve Drive Specialties - MK3 Module (Kraken X60) */
			public static COTSTalonFXSwerveConstants KrakenX60(DriveRatios driveRatio) {
				return create(driveRatio, DCMotor.getKrakenX60(1), DCMotor.getKrakenX60(1));
			}

			public enum DriveRatios {
				/** SDS MK3 - Standard (8.16 : 1) */
				STANDARD(14, 8.16),
				/** SDS MK3 - Fast (6.86 : 1) */
				FAST(14, 6.86);

				/** Teeth on the motor pinion (driving gear of first drive stage) */
				public final int drivingGearTeeth;
				/** Overall drive reduction (motor rotations per wheel rotation) */
				public final double overallGearRatio;

				DriveRatios(int drivingGearTeeth, double overallGearRatio) {
					this.drivingGearTeeth = drivingGearTeeth;
					this.overallGearRatio = overallGearRatio;
				}
			}
		}

		/** Swerve Drive Specialties - MK4 Module*/
		public static final class MK4 {

			private static final Distance WHEEL_DIAMETER = Units.Inches.of(4.0);

			/** Steer (azimuth) gear reduction for SDS MK4: motor revs per 1 azimuth rev */
			private static final double STEER_GEAR_RATIO = 12.8; // 12.8 : 1

			/** Teeth on the first-stage driven gear (SDS MK3/MK4/MK4i) */
			private static final int FIRST_STAGE_DRIVEN_TEETH = 50;

			/** CTRE definition: drive motor revs per 1 azimuth rev = inverse of first drive stage */
			private static double calculateCouplingGearRatio(int pinionTeeth) {
				if (pinionTeeth < 1) throw new IllegalArgumentException("pinionTeeth must be >= 1");
				return (double) FIRST_STAGE_DRIVEN_TEETH / pinionTeeth;
			}

			private static COTSTalonFXSwerveConstants create(DriveRatios driveRatio, DCMotor driveMotor, DCMotor steerMotor) {

				boolean driveMotorInvert = false;
				boolean steerMotorInvert = false;
				boolean cancoderInvert = false;

				return new COTSTalonFXSwerveConstants(
						WHEEL_DIAMETER,
						STEER_GEAR_RATIO,
						driveRatio.overallGearRatio,
						calculateCouplingGearRatio(driveRatio.drivingGearTeeth),
						driveMotor,
						steerMotor,
						driveMotorInvert,
						steerMotorInvert,
						cancoderInvert);
			}

			/** Swerve Drive Specialties - MK4 Module (Falcon 500) */
			public static COTSTalonFXSwerveConstants Falcon500(DriveRatios driveRatio) {
				return create(driveRatio, DCMotor.getFalcon500(1), DCMotor.getFalcon500(1));
			}

			/** Swerve Drive Specialties - MK4 Module (Kraken X60) */
			public static COTSTalonFXSwerveConstants KrakenX60(DriveRatios driveRatio) {
				return create(driveRatio, DCMotor.getKrakenX60(1), DCMotor.getKrakenX60(1));
			}

			public enum DriveRatios {
				// Standard MK4 (14T pinion)
				L1(14, 8.14), // overall reduction (motor revs : wheel rev)
				L2(14, 6.75),
				L3(14, 6.12),

				// PLUS variants (16T pinion): overall ratio scaled by 14/16 (less reduction, more free speed)
				L1_PLUS(16, 8.14 * 14.0 / 16.0), // ≈ 7.1225
				L2_PLUS(16, 6.75 * 14.0 / 16.0), // ≈ 5.90625
				L3_PLUS(16, 6.12 * 14.0 / 16.0), // ≈ 5.355

				L4(16, 5.14); // overall reduction (uses 16T pinion)

				/** Teeth on the motor pinion (driving gear of first drive stage) */
				public final int drivingGearTeeth;
				/** Overall drive reduction (motor rotations per wheel rotation) */
				public final double overallGearRatio;

				DriveRatios(int drivingGearTeeth, double overallGearRatio) {
					this.drivingGearTeeth = drivingGearTeeth;
					this.overallGearRatio = overallGearRatio;
				}
			}
		}

		/** Swerve Drive Specialties - MK4i Module*/
		public static final class MK4i {
			private static final Distance WHEEL_DIAMETER = Units.Inches.of(4.0);
			/** Teeth on the first-stage driven gear (SDS MK3/MK4/MK4i) */
			private static final int FIRST_STAGE_DRIVEN_TEETH = 50;

			/** Steer (azimuth) gear reduction: motor revs per 1 azimuth rev (SDS MK4i) */
			private static final double STEER_GEAR_RATIO = 150.0 / 7.0; // ≈ 21.4286:1

			private static double calculateCouplingGearRatio(int pinionTeeth) {
				// CTRE definition: drive motor revs per 1 azimuth rev = inverse of first drive stage
				return (double) FIRST_STAGE_DRIVEN_TEETH / pinionTeeth;
			}

			private static COTSTalonFXSwerveConstants create(DriveRatios driveRatio, DCMotor driveMotor, DCMotor steerMotor) {

				boolean driveMotorInvert = false;
				boolean steerMotorInvert = true;
				boolean cancoderInvert = false;

				return new COTSTalonFXSwerveConstants(
						WHEEL_DIAMETER,
						STEER_GEAR_RATIO,
						driveRatio.overallGearRatio,
						calculateCouplingGearRatio(driveRatio.drivingGearTeeth),
						driveMotor,
						steerMotor,
						driveMotorInvert,
						steerMotorInvert,
						cancoderInvert);
			}

			/** Swerve Drive Specialties - MK4i Module (Falcon 500) */
			public static COTSTalonFXSwerveConstants Falcon500(DriveRatios driveRatio) {
				return create(driveRatio, DCMotor.getFalcon500(1), DCMotor.getFalcon500(1));
			}

			/** Swerve Drive Specialties - MK4i Module (Kraken X60) */
			public static COTSTalonFXSwerveConstants KrakenX60(DriveRatios driveRatio) {
				return create(driveRatio, DCMotor.getKrakenX60(1), DCMotor.getKrakenX60(1));
			}

			public enum DriveRatios {
				// Standard MK4i (14T pinion)
				L1(14, 8.14), // overall reduction (motor revs : wheel rev)
				L2(14, 6.75),
				L3(14, 6.12),

				// + variants (16T pinion) — overall ratio scaled by 14/16 (less reduction, more free speed)
				L1_PLUS(16, 8.14 * 14.0 / 16.0), // ≈ 7.1225
				L2_PLUS(16, 6.75 * 14.0 / 16.0), // ≈ 5.90625
				L3_PLUS(16, 6.12 * 14.0 / 16.0); // ≈ 5.355

				/** Teeth on the motor pinion (driving gear of first drive stage) */
				public final int drivingGearTeeth;
				/** Overall drive reduction (motor rotations per wheel rotation) */
				public final double overallGearRatio;

				DriveRatios(int drivingGearTeeth, double overallGearRatio) {
					this.drivingGearTeeth = drivingGearTeeth;
					this.overallGearRatio = overallGearRatio;
				}
			}
		}
	}
}
