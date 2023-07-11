using RosMessageTypes.Geometry;
using System.Collections.Generic;
using Unity.Robotics.ROSTCPConnector;
using Unity.Robotics.ROSTCPConnector.ROSGeometry;
using UnityEngine;


public class SetRobot : MonoBehaviour
{
    ROSConnection ros;
    public string robotStateArray = "robot_state_array";
    [SerializeField] GameObject robotCalibration;
    [SerializeField] GameObject robotBase;
    [SerializeField] GameObject robot;
    [SerializeField] GameObject gripperForTracking;
    [SerializeField] ArticulationBody[] robotForMoveIt;
    [SerializeField] List<GameObject> robotJoints;
    List<GameObject> linksForROS = new List<GameObject>();
    [SerializeField] VisualizeRobot robotMovingScript;

    // Start is called before the first frame update
    void Start()
    {
        ros = ROSConnection.GetOrCreateInstance();
        ros.RegisterPublisher<PoseArrayMsg>(robotStateArray);
        findAndDeactivateUnwanted("unnamed");
        robotForMoveIt = robot.GetComponentsInChildren<ArticulationBody>();
        foreach (var el in robotForMoveIt)
        {
            robotJoints.Add(el.gameObject);
        }
         robotJoints.RemoveRange(12, robotJoints.Count -12);
        changeRobotPosition();
    }

    // Update is called once per frame
    void Update()
    {

    }

    /// <summary>
    /// Finds and deactives a gameobject with a certain name, used right now for the robot collision objects
    /// <para><paramref name="name"/> = the name of the unwanted gamobject </para>
    /// </summary>
    private void findAndDeactivateUnwanted(string name)
    {
        var potential = FindObjectsOfType<GameObject>(true);
        foreach (var item in potential)
        {
            if (item.name == name) { 
                item.gameObject.SetActive(false);
            }
        }
    }

    /// <summary>
    /// Toggles the visability of gameobjects
    ///  <para><paramref name="state"/> = true if should be active, false otherwise </para>
    ///  <para><paramref name="obj"/> = an array of gamobject </para>
    /// </summary>

    private void toggleUnwanted(bool state, GameObject[] obj)
    {
        foreach (GameObject go in obj)
        {
            go.SetActive(state);
        }
    }


   


    /// <summary>
    /// Hides the robot and shows the base link to position it
    /// </summary>
    public void changeRobotPosition()
    {
 
        robotBase.SetActive(true);
        robot.SetActive(false);
    }

    /// <summary>
    /// Set the robot to the position of the base link
    /// </summary>
    public void setRobotPosition()
    {
        robot.transform.position = robotBase.transform.position;
        robot.transform.rotation = robotBase.transform.rotation;
        robot.transform.localScale = robotBase.transform.localScale;

        gripperForTracking.transform.localScale = robotBase.transform.localScale;

        robot.SetActive(true);
        robotBase.SetActive(false);
        robotMovingScript.setNeutralPositionRobot();
        
    }


    /// <summary>
    /// Sends the position and rotation of the links to ROS, not used right now
    /// </summary>
    private void sendJointTransform()
    {
        PoseArrayMsg msg = new PoseArrayMsg();
        PoseMsg[] posesToAdd = new PoseMsg[linksForROS.Count];
        for (int i = 0; i < linksForROS.Count; i++)
        {
            PoseMsg temp = new PoseMsg();
            var pos = linksForROS[i].transform.position.To<FLU>();
            var rot = linksForROS[i].transform.rotation.To<FLU>();
            temp.position = pos;
            temp.orientation= rot;
            posesToAdd[i] = temp;
        }
        msg.poses = posesToAdd;
        ros.Publish(robotStateArray, msg);
    }



    /// <summary>
    /// Creates a ROS message with the position and orientation of the base link of the robot, not used right now
    /// </summary>
    /// <returns> the ROS message </returns> 
    TransformMsg getRobotROSLink0()
    {
       Vector3Msg pos = robotCalibration.transform.position.To<FLU>();
        QuaternionMsg rot = robotCalibration.transform.rotation.To<FLU>();

        TransformMsg msg = new TransformMsg(
                   pos,
                   rot
                   );

        return msg;

    }

}
