package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructArrayPublisher;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Subsystem;

public class PosEstimation implements Subsystem{

    Pose3d poseA = new Pose3d(10.0, 10.0, 10.0, new Rotation3d(0,0,0));
    Pose3d poseB = new Pose3d();

    StructPublisher<Pose3d> publisher = NetworkTableInstance.getDefault()
        .getStructTopic("MyPose", Pose3d.struct).publish();
    StructArrayPublisher<Pose3d> arrayPublisher = NetworkTableInstance.getDefault()
        .getStructArrayTopic("MyPoseArray", Pose3d.struct).publish();

    public PosEstimation(){}

    @Override
    public void periodic(){
        publisher.set(poseA);
        arrayPublisher.set(new Pose3d[] {poseA});
        SmartDashboard.putString("POS", poseA.toString());
        SmartDashboard.putString("test", "8808");

    }
}
