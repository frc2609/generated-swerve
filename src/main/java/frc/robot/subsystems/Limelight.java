package frc.robot.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import com.ctre.phoenix6.mechanisms.swerve.SwerveModule;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructArrayPublisher;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.LimelightHelpers;
import edu.wpi.first.math.util.Units;


public class Limelight extends SubsystemBase {
    private final NetworkTable table;

    private final NetworkTableEntry tx;
    private final NetworkTableEntry ty;
    private final NetworkTableEntry ta;
    private final NetworkTableEntry tv;
    
    private final Translation2d m_frontLeftLocation = new Translation2d(0.36195, 0.36195);
    private final Translation2d m_frontRightLocation = new Translation2d(0.36195, -0.36195);
    private final Translation2d m_backLeftLocation = new Translation2d(-0.36195, 0.36195);
    private final Translation2d m_backRightLocation = new Translation2d(-0.36195, -0.36195);

    private final SwerveDriveKinematics m_kinematics =
      new SwerveDriveKinematics(
          m_frontLeftLocation, m_frontRightLocation, m_backLeftLocation, m_backRightLocation);

    // private final SwerveDrivePoseEstimator m_poseEstimator =
        // new SwerveDrivePoseEstimator(
        //     m_kinematics,
        //     m_gyro.getRotation2d(),
        //     new SwerveModulePosition[] {
        //         m_frontLeft.getPosition(),
        //         m_frontRight.getPosition(),
        //         m_backLeft.getPosition(),
        //         m_backRight.getPosition()
        //     },
        //     new Pose2d(),
        //     VecBuilder.fill(0.05, 0.05, Units.degreesToRadians(5)),
        //     VecBuilder.fill(0.5, 0.5, Units.degreesToRadians(30)));


    // private final SwerveModule m_frontLeft = new SwerveModule(1, 2, 0, 1, 2, 3);
    // private final SwerveModule m_frontRight = new SwerveModule(3, 4, 4, 5, 6, 7);
    // private final SwerveModule m_backLeft = new SwerveModule(5, 6, 8, 9, 10, 11);
    // private final SwerveModule m_backRight = new SwerveModule(7, 8, 12, 13, 14, 15);

    private Pose2d botPos = new Pose2d();

    int[] validIDs = {3,4};
    
    private StructPublisher<Pose2d> arrayPublisher = NetworkTableInstance.getDefault()
    .getStructTopic("2Dpos", Pose2d.struct).publish();
    
    
    public Limelight() {
        LimelightHelpers.SetFiducialIDFiltersOverride("limelight", validIDs);
        table = NetworkTableInstance.getDefault().getTable("limelight");

        tx = table.getEntry("tx");
        ty = table.getEntry("ty");
        ta = table.getEntry("ta");
        tv = table.getEntry("tv");


    }

    public void updateOdometry() {
        // m_poseEstimator.update(
        //     m_gyro.getRotation2d(),
        //     new SwerveModulePosition[] {
        //         m_frontLeft.getPosition(),
        //         m_frontRight.getPosition(),
        //         m_backLeft.getPosition(),
        //         m_backRight.getPosition()
        // });


        boolean useMegaTag2 = true; //set to false to use MegaTag1
        boolean doRejectUpdate = false;

        
        // LimelightHelpers.SetRobotOrientation("limelight", m_poseEstimator.getEstimatedPosition().getRotation().getDegrees(), 0, 0, 0, 0, 0);
        // LimelightHelpers.PoseEstimate mt2 = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2("limelight");

        // m_poseEstimator.setVisionMeasurementStdDevs(VecBuilder.fill(.7,.7,9999999));
        // m_poseEstimator.addVisionMeasurement(
        //     mt2.pose,
        //     mt2.timestampSeconds);
        
    }

    
    public double getX() {
        return tx.getDouble(0.0);
    }
    
    public double getY() {
        return ty.getDouble(0.0);
    }
    
    public double getArea() {
        return ta.getDouble(0.0);
    }
    
    public void updateDashboard() {
        SmartDashboard.putNumber("LimelightX", getX());
        SmartDashboard.putNumber("LimelightY", getY());
        SmartDashboard.putNumber("LimelightArea", getArea());
        SmartDashboard.putBoolean("Target", isTargetVisible());
        SmartDashboard.putNumber("Target actual", tv.getDouble(0));
    }

    public void updatePos(){
        double[] tableData = table.getEntry("botpose_wpiblue").getDoubleArray(new double[6]);
        botPos = new Pose2d(tableData[0], tableData[1], new Rotation2d(tableData[5] * (Math.PI/180)));
        arrayPublisher.set(botPos);
    }

    public boolean isCloseToTarget() {
        double areaThreshold = 10.0;
        return getArea() >= areaThreshold;
    }

    public boolean isTargetVisible() {
        return tv.getDouble(0) == 1.0;
    }

    @Override
    public void periodic() {
        updateDashboard();
        updatePos();
    }
}