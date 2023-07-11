using RosMessageTypes.Geometry;
using RosMessageTypes.Std;
using System.Collections;
using System.Collections.Generic;
using Unity.Robotics.ROSTCPConnector;
using Unity.Robotics.ROSTCPConnector.ROSGeometry;
using UnityEngine;



public class ROS : MonoBehaviour
{
    //Topics
    ROSConnection ros;
    public string singlePose = "check_single_pose";
    public string fullPath = "check_full_path";
    public string singlePoseReply = "reachability_single_pose";
    public string fullPathReply = "reachability_full_path";
    public bool isPointReachable = true;

    //Connection status on HoloLens
    [SerializeField] ROSConnection statusROS;
    [SerializeField] GameObject statusEr;
    [SerializeField] GameObject statusErHololens;
    [SerializeField] Material connectedMaterial;
    [SerializeField] Material disconnectedMaterial;


    [SerializeField] MainController trajectoryScript;
    [SerializeField] VisualizeRobot moveRobotScript;
    [SerializeField] GameObject holderROSConnectionScript;
    [SerializeField] GameObject link0;

    //Handling full path reply
    public List<int> unreachableRange;


    // Start is called before the first frame update
    void Start()
    {
        statusROS = transform.parent.GetChild(0).GetComponent<ROSConnection>();
        ros = ROSConnection.GetOrCreateInstance();
        ros.RegisterPublisher<PoseMsg>(singlePose);
        ros.RegisterPublisher<PoseArrayMsg>(fullPath);
        ros.Subscribe<BoolMsg>(singlePoseReply, replySinglePoseReachable);
        ros.Subscribe<Int8MultiArrayMsg>(fullPathReply, replyPathReachable);

    }

    // Update is called once per frame
    void Update()
    {
        // display if ROS connection is active
        if(statusROS.HasConnectionError)
        {
            statusEr.GetComponent<Renderer>().material = disconnectedMaterial;
            statusErHololens.GetComponent<Renderer>().material = disconnectedMaterial;
        }
        else
        {
            statusEr.GetComponent<Renderer>().material = connectedMaterial;
            statusErHololens.GetComponent<Renderer>().material = connectedMaterial;

        }
    }


    /// <summary>
    /// Callback function to handle the reply if the path is reachable, gets an array wherre 1 means this position is reachable and 0 not. Converts this in an array consisiting of start and stop index of the unreachable parts
    /// <para><paramref name="msg"/> = An array of 1 and 0 encoding the reachability of each index </para>
    /// </summary>
    public void replyPathReachable(Int8MultiArrayMsg msg)
    {
        unreachableRange = new List<int>();
        int startIdx = -1;
        for (int i = 0; i < msg.data.Length; i++)
        {
            if (msg.data[i] == 1)
            {
                if (startIdx != -1)
                {
                    unreachableRange.Add(startIdx);
                    unreachableRange.Add(i - 1);
                    startIdx = -1;
                }
            }
            else if (startIdx == -1)
            {
                startIdx = i;
            }
        }

        if (startIdx != -1)
        {
            unreachableRange.Add(startIdx);
            unreachableRange.Add(msg.data.Length - 1);
        }
        trajectoryScript.showResultReachabilityPath(unreachableRange.ToArray());
    }

    /// <summary>
    /// Callback function of the result of the single pose check. Handles what needs to happen when the point is reachable or not
    /// <para><paramref name="msg"/> = A message containing true if reachable, false otherwise index </para>
    /// </summary>
    public void replySinglePoseReachable(BoolMsg msg)
    {
        if (msg != null)
        {
            if (msg.data)
            {
                isPointReachable = true;
                trajectoryScript.visualizeReachabilityTime(true);

                //EXPERIMENTAL BUG FIXING
                moveRobotScript.visualizeLastPossibleRobotPosition();

            }
            else
            {
                isPointReachable = false;
                trajectoryScript.visualizeReachabilityTime(false);
            }
        }
    }



    /// <summary>
    /// Sends the full path (positions) to ROS toc check with the IK if everything is reachable or if not, which parts.
    /// <para><paramref name="pathToCheck"/> = A list of positions which should be checked </para>
    /// </summary>

    public void checkIfPathIsReachableROS(List<Vector3> pathToCheck)
    {

        PoseArrayMsg msg = new PoseArrayMsg();
        PoseMsg[] posesToAdd = new PoseMsg[pathToCheck.Count];

        for (int i = 0; i < pathToCheck.Count; i++)
        {
            PoseMsg temp = new PoseMsg();

            Matrix4x4 relativeTRS = Matrix4x4.TRS(pathToCheck[i], Quaternion.identity, link0.transform.localScale);
            var relativeTransform = link0.transform.worldToLocalMatrix * relativeTRS;
            temp.position = relativeTransform.GetPosition().To<FLU>();
            posesToAdd[i] = temp;
        }
        msg.poses = posesToAdd;
        ros.Publish(fullPath, msg);
    }

    /// <summary>
    /// Sends a pose to check (position and rotation) to ROS to check with the IK
    /// <para><paramref name="posToCheck"/> = The position which should be checked </para>
    /// <para><paramref name="orientationGripper"/> The orientation which should be checked </para>
    /// </summary>
    public void checkCurrentPositionReachableROS(Vector3 posToCheck, Quaternion orientationGripper)
    {
        TransformMsg msg = new TransformMsg();
        Matrix4x4 relativeTRS = Matrix4x4.TRS(posToCheck, orientationGripper, link0.transform.localScale);
        
        //The position
        var relativeTransform = link0.transform.worldToLocalMatrix * relativeTRS;
        var getPosBack = relativeTransform.GetPosition();
        Vector3Msg pos = getPosBack.To<FLU>();


        //The rotation
        var rotBeforeCorrection = Quaternion.LookRotation(relativeTransform.GetColumn(2), relativeTransform.GetColumn(1));
        var rotAfterCorrection = rotBeforeCorrection * Quaternion.Euler(Vector3.up * -135);
        var rot = rotAfterCorrection.To<FLU>();

        //Creating message
        msg = new TransformMsg(
            pos,
            rot
            );
        ros.Publish(singlePose, msg);
    }

    /// <summary>
    /// Can be used for debugging, simulates a delay until the reachability of a pose is evaluated
    /// </summary>
    /// <returns></returns>
    IEnumerator delayCheckSingle()
    {
        yield return new WaitForSecondsRealtime(2);
        if (UnityEngine.Random.Range(0, 10) < 7)
        {
            isPointReachable = true;
            trajectoryScript.visualizeReachabilityTime(true);

        }
        else
        {
            isPointReachable = false;
            trajectoryScript.visualizeReachabilityTime(false);
        }
    }
}
