package com.team5449.frc2024;
import java.util.NoSuchElementException;
import java.util.function.BooleanSupplier;

import com.team5449.lib.interpolate.InterpolatingDouble;
import com.team5449.lib.interpolate.InterpolatingTreeMap;
import com.team5449.lib.util.AllianceFlipUtil;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.interpolation.TimeInterpolatableBuffer;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveWheelPositions;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;



public class RobotState {
    private BooleanSupplier lookaheadDisable;


    public record AimingParameters
    (Rotation2d driveHeading,
    Rotation2d armAngle,
    double effectiveDistance,
    double driveFeedVelocity) {
    }

    public record OdometryObservation(
        SwerveDriveWheelPositions wheelPositions, Rotation2d gyroAngle, double timestamp
    ) {}

    public record VisionObservation(Pose2d visionPose, double timestamp, Matrix<N3, N1> stdDevs) {
    }

    private static final InterpolatingTreeMap<InterpolatingDouble, InterpolatingDouble> mArmTreeMap = new InterpolatingTreeMap<>();

    static{
        
    }

    private static RobotState mInstance;
    public static RobotState getInstance(){
        if(mInstance == null) mInstance = new RobotState();
        return mInstance;
    }

    private Pose2d odometryPose = new Pose2d();
    private Pose2d estimatedPose = new Pose2d();
    private final double poseBufferSizeSeconds = 2.0;
    private final TimeInterpolatableBuffer<Pose2d> poseBuffer = TimeInterpolatableBuffer.createBuffer(2.0);
    private final Matrix<N3, N1> qStdDevs = new Matrix<>(Nat.N3(), Nat.N1());
    private final SwerveDriveKinematics kinematics;
    private BooleanSupplier lookahead = () -> false;
    private SwerveDriveWheelPositions lastWheelPositions = 
        new SwerveDriveWheelPositions(
            new SwerveModulePosition[] {
                new SwerveModulePosition(),
                new SwerveModulePosition(),
                new SwerveModulePosition(),
                new SwerveModulePosition()
            }
        );

    private Rotation2d lastGyroAngle = new Rotation2d();
    private Twist2d robotVelocity = new Twist2d();
    private AimingParameters latestParameters = null;


    private RobotState(){
        for(int i = 0; i < 3; ++i){
            qStdDevs.set(i, 0, Math.pow(Constants.odometryStateStdDevs.get(i, 0), 2));
        }

        kinematics = Constants.swerveDriveKinematics;
    }

    public void addOdometryObservation(OdometryObservation observation){
        latestParameters = null;
        Twist2d twist = kinematics.toTwist2d(lastWheelPositions, observation.wheelPositions());
        lastWheelPositions = observation.wheelPositions();
        // Check gyro connected
        if (observation.gyroAngle != null) {
        // Update dtheta for twist if gyro connected
        twist =
            new Twist2d(
                twist.dx, twist.dy, observation.gyroAngle().minus(lastGyroAngle).getRadians());
        lastGyroAngle = observation.gyroAngle();
        }
        // Add twist to odometry pose
        odometryPose = odometryPose.exp(twist);
        // Add pose to buffer at timestamp
        poseBuffer.addSample(observation.timestamp(), odometryPose);
        // Calculate diff from last odometry pose and add onto pose estimate
        estimatedPose = estimatedPose.exp(twist);
    }


    public Pose2d getEstimatedPose(){
        return estimatedPose;
    }

  public void addVisionObservation(VisionObservation observation) {
    latestParameters = null;
    // // If measurement is old enough to be outside the pose buffer's timespan, skip.
    try {
      if (poseBuffer.getInternalBuffer().lastKey() - poseBufferSizeSeconds
          > observation.timestamp()) {
        return;
      }
    } catch (NoSuchElementException ex) {
      return;
    }
    // Get odometry based pose at timestamp
    var sample = poseBuffer.getSample(observation.timestamp());
    if (sample.isEmpty()) {
      // exit if not there
      return;
    }

    // sample --> odometryPose transform and backwards of that
    var visionSampleToOdometry = new Transform2d(sample.get(), odometryPose);
    var odometryToVisionSample = new Transform2d(odometryPose, sample.get());
    // get old estimate by applying odometryToSample Transform
    Pose2d estimateAtTime = estimatedPose.plus(odometryToVisionSample);

    // Calculate 3 x 3 vision matrix
    var r = new double[3];
    for (int i = 0; i < 3; ++i) {
      r[i] = Math.pow(observation.stdDevs().get(i, 0),2);
    }
    // Solve for closed form Kalman gain for continuous Kalman filter with A = 0
    // and C = I. See wpimath/algorithms.md.
    Matrix<N3, N3> visionK = new Matrix<>(Nat.N3(), Nat.N3());
    for (int row = 0; row < 3; ++row) {
      double stdDev = qStdDevs.get(row, 0);
      if (stdDev == 0.0) {
        visionK.set(row, row, 0.0);
      } else {
        visionK.set(row, row, stdDev / (stdDev + Math.sqrt(stdDev * r[row])));
      }
    }
    // difference between estimate and vision pose
    Transform2d transform = new Transform2d(estimateAtTime, observation.visionPose());
    // scale transform by visionK
    var kTimesTransform =
        visionK.times(
            VecBuilder.fill(
                transform.getX(), transform.getY(), transform.getRotation().getRadians()));
    Transform2d scaledTransform =
        new Transform2d(
            kTimesTransform.get(0, 0),
            kTimesTransform.get(1, 0),
            Rotation2d.fromRadians(kTimesTransform.get(2, 0)));

    // Recalculate current estimate by applying scaled transform to old estimate
    // then replaying odometry data
    estimatedPose = estimateAtTime.plus(scaledTransform).plus(visionSampleToOdometry);
    }

    

    //public AimingParameters getAimingParameters(){
    //     if (latestParameters != null) {
    //   // Cache previously calculated aiming parameters. Cache is invalidated whenever new
    //   // observations are added.
    //   return latestParameters;
    // }

    // Transform2d fieldToTarget =
    //     new Transform2d(AllianceFlipUtil.apply(FieldConstants.Speaker.centerSpeakerOpening)
    //         .toTranslation2d(), new Rotation2d());

    // Pose2d fieldToPredictedVehicle =
    //     lookaheadDisable.getAsBoolean()
    //         ? getEstimatedPose()
    //         : getPredictedPose(lookahead.get(), lookahead.get());
    // Pose2d fieldToPredictedVehicleFixed =
    //     new Pose2d(fieldToPredictedVehicle.getTranslation(), new Rotation2d());

    // Translation2d predictedVehicleToTargetTranslation =
    //     fieldToPredictedVehicle.inverss().transformBy(fieldToTarget).getTranslation();
    // Translation2d predictedVehicleFixedToTargetTranslation =
    //     fieldToPredictedVehicleFixed.unaryMinus().transformBy(fieldToTarget).getTranslation();

    // Rotation2d vehicleToGoalDirection = predictedVehicleToTargetTranslation.getAngle();

    // Rotation2d targetVehicleDirection = predictedVehicleFixedToTargetTranslation.getAngle();
    // double targetDistance = predictedVehicleToTargetTranslation.getNorm();

    // double feedVelocity =
    //     robotVelocity.dx * vehicleToGoalDirection.getSin() / targetDistance
    //         - robotVelocity.dy * vehicleToGoalDirection.getCos() / targetDistance;

    // latestParameters =
    //     new AimingParameters(
    //         targetVehicleDirection,
    //         Rotation2d.fromDegrees(mArmTreeMap.getInterpolated(new InterpolatingDouble(targetDistance)).value + 0.0),
    //         targetDistance,
    //         feedVelocity);
    // return latestParameters;
    // }


}
