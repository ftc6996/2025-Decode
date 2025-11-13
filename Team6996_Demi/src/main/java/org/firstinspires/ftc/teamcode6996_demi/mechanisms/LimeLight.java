package org.firstinspires.ftc.teamcode6996_demi.mechanisms;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;

import java.util.List;

public class LimeLight {

    // Dont forget to run Limelight Hardware Manager
    // IP Address http://172.28.0.1:5801/
    private Limelight3A vision;
    private LLResult result;

    //Store the information of the april tag we are tracking
    private int tagID;
    private double tagLocationX, tagLocationY, tagDistance;
    private double tagLocationArea, tagLocationWidth, tagLocationHeight;
    private double angleToGoal; //in radians
    public LimeLight(Limelight3A vision)
    {
        this.vision = vision;

        setPipeline(0);
        vision.start();
        clearFoundAprilTag();
    }

    public LimeLight(Limelight3A vision, int pipeline)
    {
        this.vision = vision;

        setPipeline(pipeline);
        vision.start();
        clearFoundAprilTag();
    }

    public void setPipeline(int pipeline)
    {
        vision.pipelineSwitch(pipeline);
    }

    public int getPipeline()
    {
        return result.getPipelineIndex();
    }

    public void clearFoundAprilTag()
    {
        tagID = 0;
        tagLocationX = 0;
        tagLocationY = 0;
        tagLocationArea = 0;
        tagLocationWidth = 0;
        tagLocationHeight = 0;
        tagDistance = 0;
    }
    public void getAprilTags()
    {
        result = vision.getLatestResult();

        List<LLResultTypes.FiducialResult> fiducials = result.getFiducialResults();
        for (LLResultTypes.FiducialResult fiducial : fiducials) {
            tagID = fiducial.getFiducialId();
            tagLocationX = fiducial.getTargetXDegrees();
            tagLocationY = fiducial.getTargetYDegrees();
            tagLocationWidth = fiducial.getTargetXPixels();
            tagLocationHeight = fiducial.getTargetYPixels();
            tagLocationArea = tagLocationWidth * tagLocationHeight;
            //AprilTagdistance = getTargetY(Measurement.INCHES) / Math.tan(angleToGoalRadian); // Find distance to fiducial
        }
        angleToGoal = Math.toRadians(0 + tagLocationY);
    }
}
