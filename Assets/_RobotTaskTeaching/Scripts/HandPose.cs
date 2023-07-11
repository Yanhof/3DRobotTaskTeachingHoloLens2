using UnityEngine;
using Microsoft.MixedReality.Toolkit.Input;
using Microsoft.MixedReality.Toolkit.Utilities;



public class HandPose : MonoBehaviour
{
    //Parameters to determine which sign is shown
    Handedness handsToTrack = Handedness.Left; // change to test easier on PC to right
    const float PinchThreshold = 0.7f;
    const float curlFingerThershold = 0.7f;
    const float curlThumbThreshold = 0.3f;
    const float curlThumbVictory = 0.3f;
    const float curlFingerVictory = 0.4f;
    const float curlFingersVictoryStraight = 0.3f;

    //Bools for detected sign
    public bool isPinchingTrue = false;
    public bool isThumbsUpTrue = false;
    public bool isVictoryTrue = false;

    //Time variables
    float lastTimePinch = 0.0f;
    float lastTimeThumb = 0.0f;
    float lastTimeVictory = 0.0f;
    float cooldownTimeThumb = 1f;
    float cooldownTimePinch = 0.8f;
    float cooldownTimeVictory = 1.2f;


    private void Start()
    {
       
        lastTimeThumb = -cooldownTimeThumb; // can call start trajectory immediately
    }


    void FixedUpdate()
    {


        if (HandJointUtils.TryGetJointPose(TrackedHandJoint.IndexKnuckle, handsToTrack, out var pose))
        {

            if (timeDifference("pinch") > cooldownTimePinch && isPinching(handsToTrack))
            {
                lastTimePinch = Time.realtimeSinceStartup;
                isPinchingTrue = true;

            }
            else if (timeDifference("thumb") > cooldownTimeThumb && isThumbsUp(handsToTrack))
            {
                lastTimeThumb = Time.realtimeSinceStartup;
                isThumbsUpTrue = true;

            }

            else if (timeDifference("victory") > cooldownTimeVictory && isVictory(handsToTrack))
            {
                isVictoryTrue = true;
                lastTimeVictory = Time.realtimeSinceStartup;
                lastTimePinch= Time.realtimeSinceStartup - 0.5f;
            }

            else
            {
                isThumbsUpTrue = false;
                isPinchingTrue = false;
                isVictoryTrue = false;
            }
        }
    }


    /// <summary>
    /// Returns the time since the last time this sign was detected
    /// </summary>
    /// <para><paramref name="name"/> = The name of the sign for which the check should be done </para>
    /// <returns></returns>
    private float timeDifference(string name)
    {
        if (name == "pinch")
        {
            return Time.realtimeSinceStartup - lastTimePinch;
        }
        else if (name == "thumb")
        {
            return Time.realtimeSinceStartup - lastTimeThumb;
        }
        else if (name == "victory")
        {
            return Time.realtimeSinceStartup - lastTimeVictory;
        }
        return 0f;
    }


    /// <summary>
    /// Returns true if the victory sign is detected
    /// </summary>
    /// <para><paramref name="trackedHand"/> = on which hand the sign can be detected </para>
    /// <returns></returns>
    private bool isVictory(Handedness trackedHand)
    {
        if (HandPoseUtils.IndexFingerCurl(trackedHand) < curlFingersVictoryStraight && HandPoseUtils.MiddleFingerCurl(trackedHand) < curlFingersVictoryStraight)
        {
            if (HandPoseUtils.ThumbFingerCurl(trackedHand) > curlThumbVictory)
            {
                if (HandPoseUtils.RingFingerCurl(trackedHand) > curlFingerVictory && HandPoseUtils.PinkyFingerCurl(trackedHand) > curlFingerVictory)
                {
                    return true;
                }
            }
        }
        return false;
    }


    /// <summary>
    /// Returns true if a pinch is detected
    /// </summary>
    /// <para><paramref name="trackedHand"/> = on which hand the sign can be detected </para>
    /// <returns></returns>
    private bool isPinching(Handedness trackedHand)
    {
        if (HandPoseUtils.CalculateIndexPinch(trackedHand) > PinchThreshold)
        {
             return true;
        }
        return false;


    }


    /// <summary>
    /// Returns true if thumbs-up is detected
    /// </summary>
    /// <para><paramref name="trackedHand"/> = on which hand the sign can be detected </para>
    /// <returns></returns>
    private bool isThumbsUp(Handedness trackedHand)
    {
        if (HandPoseUtils.ThumbFingerCurl(trackedHand) < curlThumbThreshold)
        {
            if (HandPoseUtils.IndexFingerCurl(trackedHand) > curlFingerThershold && HandPoseUtils.RingFingerCurl(trackedHand) > curlFingerThershold && HandPoseUtils.MiddleFingerCurl(trackedHand) > curlFingerThershold && HandPoseUtils.PinkyFingerCurl(trackedHand) > curlFingerThershold)
            {
                return true;
            }
        }
        return false;
    }


}


