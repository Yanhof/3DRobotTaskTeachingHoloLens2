using RosMessageTypes.Sensor;
using System.Collections.Generic;
using Unity.Robotics.ROSTCPConnector;
using UnityEngine;

public class VisualizeRobot : MonoBehaviour
{
    ROSConnection ros;
    public string jointAnglesTopic = "joint_angles";
    public List<float> jointAngles = new List<float>(9);//the robot has 7 joints (and 2 fingers) which can be moved with the controller with the angles from ROS (no gripper open/close at the moment, is always open (value 0.02 for finger, for close set to 0)
    [SerializeField] ArticulationBody jointRoot;    
    [SerializeField] private ArticulationBody[] robotJoints = new ArticulationBody[7];
    [SerializeField] ROS rosReachabilityScript;
    [SerializeField] MainController trajectoryScript;
    

    // Start is called before the first frame update
    void Start()
    {
        ros = ROSConnection.GetOrCreateInstance();
        ros.Subscribe<JointStateMsg>(jointAnglesTopic, saveLastJointAngles);
        for(int i = 0;i<9;i++)
        {
            jointAngles.Add(0);
        }
        setNeutralPositionRobot();
     
    }

    // Update is called once per frame
    void Update()
    {
    }

    /// <summary>
    /// Sets the joint angles such that the robot is in a neutral position and moves the robot to it
    /// </summary>
    public void setNeutralPositionRobot()
    {
        jointAngles[0] = jointAngles[1] = jointAngles[4] = 0;
        jointAngles[2] = 3.2f;
        jointAngles[3] = -1;
        jointAngles[5] = 1;
        jointAngles[6] = 1;
        jointAngles[7] = 0.01f;
        jointAngles[8] = 0.01f;

        for (int i = 0; i < jointAngles.Count; i++)
        {
           robotJoints[i].jointPosition = new ArticulationReducedSpace(jointAngles[i]);
        }
        jointRoot.SetDriveTargets(jointAngles);

    }




    /// <summary>
    /// Saves the incoming joint angles from ROS and triggers the visualization of the robot if needed
    /// <para><paramref name="joints"/> = The value each joint has to be set to (in radians) </para>
    /// </summary>
    private void saveLastJointAngles(JointStateMsg joints)
    {
        for (int i = 0; i < 7; i++)
        {
            jointAngles[i] = (float)joints.position[i];
            
        }
        //Handle the visualiziation in the time mode
        if(trajectoryScript.modeTracking == 1 && rosReachabilityScript.isPointReachable)
        {
            visualizeLastPossibleRobotPosition();
        }

    }

    /// <summary>
    /// Moves the robot to display the last saved joint configuration received from ROS
    /// </summary>
    public void visualizeLastPossibleRobotPosition()
    {

        //rotates the joints (arm) to the wanted position, joint angles are in radians, the angles for the fingers (the last two) are 0 for close and 0.025 for max open 
       // use this for the PC
         jointRoot.SetDriveTargets(jointAngles);

        //This works for the HoloLens, on PC only quickly snaps to that position
        for (int i = 0; i < jointAngles.Count; i++)
        {
            robotJoints[i].jointPosition = new ArticulationReducedSpace(jointAngles[i]);
        }
    }
}
