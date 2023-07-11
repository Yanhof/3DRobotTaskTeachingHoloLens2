using Microsoft.MixedReality.Toolkit.Input;
using Microsoft.MixedReality.Toolkit.Utilities;
using System.Collections;
using UnityEngine;


public class GripperAtHand : MonoBehaviour
{
    //General handtracking 
    Handedness handForGripper = Handedness.Right;
    MixedRealityPose pose;
    TrackedHandJoint idealTrackedJoint = TrackedHandJoint.IndexKnuckle;
    TrackedHandJoint secondTrackedJoint = TrackedHandJoint.PinkyKnuckle;
    [SerializeField] GameObject gripper;
    [SerializeField] GameObject buttons;
    [SerializeField] GameObject managerTrajectoryRecording;
    [SerializeField] GameObject holderLastValidGripper;
    public bool firstAttachGripper = true;

    //Variables for reattaching
    int maxAngelDifferenceAttaching = 30;
    float distanceForReataching = 0.025f;
    public Vector3 lastVisibleGripperPosition = Vector3.zero;
    Quaternion correctionRotationGripper = Quaternion.Euler(0, 0, 45);
    [SerializeField] Material gripperAttached;
    [SerializeField] Material gripperLost;
    [SerializeField] Material gripperOnlyCorrectPosition;
    [SerializeField] GameObject gipperStatusIndicator;
    GameObject lastGripperObj;
    [SerializeField] MainController recordingScript;



    // Start is called before the first frame update
    void Start()
    {
        gripper.SetActive(true);

    }

    // Update is called once per frame
    void FixedUpdate()
    {

        // used to attach gripper to hand of user
        if (managerTrajectoryRecording.GetComponent<MainController>().recordingActive)
        {
            if ((gripper.activeSelf || firstAttachGripper)) {
                setRobotHeadToHand();

            }
            if (gripper.activeSelf == false)
            {
                reattachGripperToHand();
            }                  
        }
        else
        {
            StartCoroutine(destoryAllChildren(holderLastValidGripper));
            gripper.SetActive(false); 
        }


    }

    private void attachMenueTohand(Vector3 pos)
    {
        
        buttons.transform.position = pos;
    }

/// <summary>
/// Attaches the gripper to the hand for tracking 
/// </summary>
/// <returns> True if attaching was possible </returns>
    public bool setRobotHeadToHand()
    {
        if (HandJointUtils.TryGetJointPose(idealTrackedJoint, handForGripper, out pose))
        {
            firstAttachGripper = false;
            gripper.transform.position = pose.Position;
            gripper.transform.rotation = pose.Rotation * correctionRotationGripper;
            gripper.SetActive(true);
            lastVisibleGripperPosition = pose.Position;
            gipperStatusIndicator.GetComponent<Renderer>().material = gripperAttached;
            return true;
        }
        else if (HandJointUtils.TryGetJointPose(secondTrackedJoint, handForGripper, out pose))
        {
            firstAttachGripper = false;
            gripper.transform.position = pose.Position;
            gripper.transform.rotation = pose.Rotation * correctionRotationGripper;
            gripper.SetActive(true);
            lastVisibleGripperPosition = pose.Position;
            gipperStatusIndicator.GetComponent<Renderer>().material = gripperAttached;
            return true;
        }
        else
        {
            gripper.SetActive(false);
            setLastValidGripper(lastVisibleGripperPosition, gripper.transform.rotation);
            return false;
        }
    }

    /// <summary>
    /// Sets the gripper to the given position and rotation and makes it the last valid gripper position
    /// </summary>
    /// <para><paramref name="lastPos"/> = The position to set it to </para>
    /// <para><paramref name="rot"/> = The rotation to set it to </para>
    public void setLastValidGripper(Vector3 lastPos, Quaternion rot)
    {
        gipperStatusIndicator.GetComponent<Renderer>().material = gripperLost;
        holderLastValidGripper.transform.position = lastPos;

        if (holderLastValidGripper.transform.childCount == 0)
        {
            lastGripperObj = Instantiate(gripper, holderLastValidGripper.transform);
            lastGripperObj.transform.position = lastPos;
            lastGripperObj.transform.rotation = rot;
            lastGripperObj.SetActive(true);
            lastVisibleGripperPosition = lastPos;
        }
    }

    /// <summary>
    /// Tries to reattach the gripper again to the hand, compares for it the position of the hand and last valid gripper as well as the difference in rotation. True if the difference is not too large, the values of the variables for the difference can be changed on the top. Also changes the color accordingly.
    /// </summary>
    private void reattachGripperToHand()
    {

        if (HandJointUtils.TryGetJointPose(idealTrackedJoint, handForGripper, out pose))
        {
            if (Vector3.Distance(pose.Position, lastVisibleGripperPosition) < distanceForReataching) {

                lastGripperObj.transform.GetChild(0).transform.GetChild(0).GetComponent<Renderer>().material = gripperOnlyCorrectPosition;
                if (handRoughlySameRotationAsGameobject(lastGripperObj, maxAngelDifferenceAttaching))
                {

                    gripper.transform.position = pose.Position;
                    gripper.SetActive(true);
                    lastVisibleGripperPosition = gripper.transform.position;
                    Destroy(holderLastValidGripper.transform.GetChild(0).gameObject);
                    firstAttachGripper = true;
                    gipperStatusIndicator.GetComponent<Renderer>().material = gripperAttached;
                    recordingScript.reattachedAfterContinue = true;


                }
                
            }
            else
            {
                lastGripperObj.transform.GetChild(0).transform.GetChild(0).GetComponent<Renderer>().material = gripperLost;
            }
            
        }
    }


    /// <summary>
    /// Compares the rotation of a gameobject to the hand used for the gripper, special version for the gripper because of the 45 degree rotation
    /// </summary>
    /// <para><paramref name="obj"/> = The gamobject to compare </para>
    /// <para><paramref name="angleDiff"/> = The maximal allowed difference for the angle </para>
    /// <returns> True if the rotation is below the maximal angle</returns>
    private bool handRoughlySameRotationAsGameobject(GameObject obj, float angleDiff)
    {
        if(HandJointUtils.TryGetJointPose(idealTrackedJoint, handForGripper, out pose)) {
            //special for gripper
            var rotationGripper = obj.transform.rotation * Quaternion.Euler(0,0,-45);
            // end special
            
            if (Quaternion.Angle(pose.Rotation, rotationGripper) < angleDiff) {
                return true;
            }
        }
        return false;
    }

    /// <summary>
    /// Gives you the index knuckle position
    /// </summary>
    /// <returns>The index knuckle position, (0,0,0) if index knuckle is not visible</returns>
    public Vector3 getIndexKnucklePosition()
    {
        if (HandJointUtils.TryGetJointPose(idealTrackedJoint, handForGripper, out pose))
        {
            return pose.Position;
        }
        return new Vector3(0,0,0);


    }

    /// <summary>
    /// Destroys all child gamobjects of a parent gameobject
    /// </summary>
    /// <para><paramref name="par"/> = The parent gameobject whose childs should be destroyed </para>
    /// <returns></returns>
    private IEnumerator destoryAllChildren(GameObject par)
    {
        while (par.transform.childCount > 0)
        {
            Destroy(par.transform.GetChild(0).gameObject);
            yield return null;
        }
    }

}


