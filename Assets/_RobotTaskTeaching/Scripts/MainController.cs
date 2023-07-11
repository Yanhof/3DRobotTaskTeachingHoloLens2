using System.Collections.Generic;
using UnityEngine;
using Microsoft.MixedReality.Toolkit.UI;
using BezierSolution;
using Unity.VisualScripting;
using Microsoft.MixedReality.Toolkit.Input;
using System.Linq;
using System;
using System.Collections;


public class MainController : MonoBehaviour
{
    [Space]
    [Header(" ============ Creating a path ============")]
    [Space]
    [Space]
    bool trackingNow = false;
    bool firstTrack = false;
    bool showLastTrack = false;
    bool trackOnPlane = false;
    int numberOfTracksSinceStart = 0;
    Vector3 startPosition = Vector3.zero;
    Vector3 currentPosition = Vector3.zero;
    Vector3 lastPosition = Vector3.zero;
    float offset = 0.02f;
    float trackingRadius = 0.002f; //change if need more/less
    bool convertToSplineHappened = false;
    [SerializeField] List<Vector3> recordedPath;
    [SerializeField] List<Quaternion> recordedOrientation; // needed for the ROS check in IK
    [SerializeField] List<Quaternion> recordedOrientationForReattaching; // needed to set the gripper to the same orientation
    Vector3[] pointToPlot;
    public GameObject trackingPoint;
    public GameObject trackObj;
    public GameObject currentPlane;
    [SerializeField] private GameObject trackObjRecorded;
    public int modeTracking = 0; // 0 for fixed distance and 1 for time mode
    float startTimeTracking = 0.0f;
    List<Vector3> pointsToCheck;
    float timeLastPointAdded = 0.0f;
    float timeBetweenPointsAdded = 0.05f;
    float baseRadiusActivateNear;
    float radiusActivateNearPoint;
    public bool reattachedAfterContinue = true;
    [SerializeField] GameObject gripperOrientation;
    [SerializeField] GameObject gripperOrientationForReattaching;


    [Space]
    [Header(" ============ Recording of events ============")]
    [Space]
    [Space]
    [SerializeField] HandPose handPoseDetection;
    List<robotEvent> recordedEvents = new List<robotEvent>();
    public struct robotEvent
    {
        public string eventName;
        public Vector3 position;
        public Quaternion rotationGripper;

    }
    [SerializeField] GameObject eventVisualizer;
    [SerializeField] GameObject eventHolder;
    bool objectPickedUp = false;
    List<int> indexEvents = new List<int>();
    [SerializeField] GameObject gripperEvent;

    //additional hand gestures
    public bool recordingActive = false;
    [SerializeField] GameObject gripperAtHand;
    [SerializeField] GripperAtHand setGripperToHandSkript;


    [Space]
    [Header(" ============ For Spline ============")]
    [Space]
    [Space]
    LineRenderer displayedPath;
    [SerializeField] private BezierSpline spline;
    [SerializeField] private Material materialUncheckedControlpoint;

    [SerializeField] GameObject splinePointsHolder;
    public List<Vector3> generatedPointsFromSpline;
    List<robotEvent> savedEvents = new List<robotEvent>();
    float glitchDistance = 0.02f;
    [SerializeField] int intLen;
    [SerializeField] List<int> controlPointsIndices = new List<int>();


    [Space]
    [Header(" ============ Clickable menues ============")]
    [Space]
    [Space]
    [SerializeField] GameObject[] buttonsNotClickableWhileTracking;
    [SerializeField] GameObject[] buttonsInModeEquidistance;
    [SerializeField] GameObject[] buttonsInModeTime;
    [SerializeField] GameObject[] buttonsModeSelection;
    // for PC menue 
    [SerializeField] GameObject[] buttonsInModeEquidistancePC;
    [SerializeField] GameObject[] buttonsInModeTimePC;
    [SerializeField] GameObject[] buttonsModeSelectionPC;


    [Space]
    [Header(" ============ ROS ============")]
    [Space]
    [Space]
    [SerializeField] ROSImitation rosServer;
    int timeBetweenChecks = 2;
    float timeLastCheck = 0;
    Vector3 lastValidPosition = Vector3.zero;
    [SerializeField] private Material materialErrorControlpoint;
    [SerializeField] private Material materialCorrectControlpoint;
    [SerializeField] GameObject markerValidPosition;
    [SerializeField] GameObject checkedPositionsHolder;
    int indexLastChecked = 0;
    int indexLastSuccesfullCheck = 0;
    float distanceCP = 0.007f;
    [SerializeField] ROS ROSConnectionScript;
    [SerializeField] VisualizeRobot moveRobotToPosition;


    [Space]
    [Header(" ============ Saving ============")]
    [Space]
    [Space]
    [SerializeField] LineRenderer savedWorkflowPath;
    List<savedSnippets> savedWorkflow = new List<savedSnippets>();
    [SerializeField] bool addSnippetAtEnd; // true then added at the end if false at beginning
    private struct savedSnippets
    {
        public int mode;
        public List<Vector3> position;
        public List<Quaternion> orientation;
        public List<robotEvent> events;
        public List<Quaternion> orientationForReattaching;

    }
    Vector3[] savedPos;
    [SerializeField] GameObject startPointWorkflow;
    [SerializeField] GameObject endPointWorkflow;
    [SerializeField] Material materialEnd;
    [SerializeField] Material materialStart;
    [SerializeField] Material materialCurrent;
    Vector3 positionContinueTracking = Vector3.zero;
    Quaternion rotationContinueTracking = Quaternion.identity;
    bool toggleVisabilityWorkflow = true;



    // Start is called before the first frame update
    void Start()
    {
        backToModeSelector();
        //needed for continuation of workflow at one of the endpoints
        baseRadiusActivateNear = endPointWorkflow.GetComponent<Renderer>().bounds.extents.magnitude;
        radiusActivateNearPoint = baseRadiusActivateNear * 3;
    }

    // Update is called once per frame
    void FixedUpdate()
    {

        if (spline.endPoints.Count > 0)
        {
            spline.AutoConstructSpline();
            spline.Refresh();
        }

        //starts a new tracking session
        if (recordingActive == false && handPoseDetection.isThumbsUpTrue && buttonsModeSelection[0].activeSelf == false)
        {

            timeLastCheck = 0;
            visabilityRecordedTrack(false);
            startTimeTracking = Time.realtimeSinceStartup;
            enableTracking();
            isClickable(buttonsNotClickableWhileTracking, false);
        }
        //stops a tracking session
        if (Time.realtimeSinceStartup > startTimeTracking + 1 && recordingActive && handPoseDetection.isThumbsUpTrue)
        {
            disableTracking();
            isClickable(buttonsNotClickableWhileTracking, true);
        }

        if (trackingNow)
        {
            // if near the point where you want to add a snippet make sure grippper is at this position
            if (reattachedAfterContinue == false && Vector3.Distance(positionContinueTracking, setGripperToHandSkript.getIndexKnucklePosition()) < radiusActivateNearPoint)
            {
                trackObjRecorded.GetComponent<LineRenderer>().enabled = false; 
                trackingPos(trackOnPlane);              
                setGripperToHandSkript.setLastValidGripper(positionContinueTracking, rotationContinueTracking);
                gripperAtHand.SetActive(false);

            }



            if (gripperAtHand.activeSelf)
            {
                trackObjRecorded.GetComponent<LineRenderer>().enabled = true;
                trackingPos(trackOnPlane);
                //check if reachable every couple of seconds
                if (modeTracking == 1 && Time.realtimeSinceStartup - timeLastCheck > timeBetweenChecks)
                {
                    timeLastCheck = Time.realtimeSinceStartup;
                    checkROS(recordedPath);

                    #region Old ROS
                    // start old handling with ROS imitation and immediate feedback
                    /*
                     if (checkROS(recordedPath))
                     {

                         indexLastChecked = recordedPath.Count - 1;
                         Debug.Log("lastIndex checked is " + indexLastChecked);
                     }
                     else
                     {

                         recordedPath.RemoveRange(indexLastChecked + 1, recordedPath.Count - indexLastChecked - 1);
                         recordedOrientation.RemoveRange(indexLastChecked + 1, recordedOrientation.Count - indexLastChecked - 1);

                         for (int i = 0; i < indexEvents.Count; i++)
                         {
                             Debug.Log("index has value " + i);

                             if (indexEvents[i] > indexLastChecked)
                             {
                                 indexEvents.RemoveAt(i);
                                 recordedEvents.RemoveAt(i);
                                 i--;
                                 Destroy(eventHolder.transform.GetChild(eventHolder.transform.childCount - 1).gameObject);

                             }

                         }
                         Debug.Log("Number of recordedEvents is " + recordedEvents.Count);

                         setGripperToHandSkript.lastVisibleGripperPosition = recordedPath[indexLastChecked];
                         gripperAtHand.SetActive(false);
                         setGripperToHandSkript.setLastValidGripper(recordedPath[indexLastChecked], recordedOrientation[indexLastChecked]);
                         setGripperToHandSkript.firstAttachGripper = false;

                         // end old handling with ROS imitation and immediate feedback
                     }
                     */
                    #endregion
                }
            }
            else
            {
                timeLastCheck = Time.realtimeSinceStartup; // time for checks does not run as long as hand is not visible (like paused)
                timeLastPointAdded = Time.realtimeSinceStartup; // time for checks does not run as long as hand is not visible (like paused)
            }
        }
        if (showLastTrack)
        {
            showTrack();
            splineEditing();
        }
    }


    /// <summary>
    /// Handles what is visible in the mode selection menue
    /// </summary>
    public void backToModeSelector()
    {
        setGameobjectsInArrayActive(buttonsInModeEquidistance, false);
        setGameobjectsInArrayActive(buttonsInModeTime, false);
        setGameobjectsInArrayActive(buttonsModeSelection, true);

        //only for PC mode
        setGameobjectsInArrayActive(buttonsInModeEquidistancePC, false);
        setGameobjectsInArrayActive(buttonsInModeTimePC, false);
        setGameobjectsInArrayActive(buttonsModeSelectionPC, true);


    }

    /// <summary>
    /// Sets all gameobjects either active or inactive
    /// </summary>
    /// <para><paramref name="obj"/> = An array of gameobjects </para>
    /// <para><paramref name="active"/> = The state to set them to</para>
    private void setGameobjectsInArrayActive(GameObject[] obj, bool active)
    {
        foreach (var el in obj)
        {
            el.SetActive(active);
        }
    }


    /// <summary>
    /// Makes gameobjects clickable (or not)
    /// </summary>
    /// <para><paramref name="obj"/> = An array of gameobjects </para>
    /// <para><paramref name="clickable"/> = The state to set them to</para>
    private void isClickable(GameObject[] obj, bool clickable)
    {
        foreach (var el in obj)
        {
            el.GetComponentInChildren<Interactable>().enabled = clickable;
        }
    }

    /// <summary>
    /// Makes the necessary adjustments for the choosen tracking mode
    /// </summary>
    /// <para><paramref name="mode"/> = The mode, 1 for time mode and 0 for equidistance mode</para>
    public void setModeTracking(int mode)
    {
        modeTracking = mode;
        setGameobjectsInArrayActive(buttonsModeSelection, false);
        //for PC
        setGameobjectsInArrayActive(buttonsModeSelectionPC, false);

        if (mode == 0)
        {
            intLen = (int)(0.06f / trackingRadius);
            setGameobjectsInArrayActive(buttonsInModeEquidistance, true);
            //for PC
            setGameobjectsInArrayActive(buttonsInModeEquidistancePC, true);

        }
        else if (mode == 1)
        {
            setGameobjectsInArrayActive(buttonsInModeTime, true);
            //for PC
            setGameobjectsInArrayActive(buttonsInModeTimePC, true);
        }
    }



    /// <summary>
    /// Shows (or hides) all the recorded events
    /// </summary>
    /// <para><paramref name="holder"/> = Where they are stored</para>
    /// <para><paramref name="visible"/> = The state to set them to</para>

    private void showEvents(GameObject holder, bool visible)
    {
        if (visible)
        {
            holder.SetActive(visible);
            foreach (robotEvent eventData in recordedEvents)
            {
                Debug.Log($"Event name: {eventData.eventName}, Position: {eventData.position}, Rotation: {eventData.rotationGripper}");
            }
        }
    }

    /// <summary>
    /// Visualizes all saved events
    /// </summary>
    private void visualizeAllEvent()
    {

        foreach (robotEvent eventData in recordedEvents)
        {
            var curEvent = Instantiate(eventVisualizer, eventHolder.transform);
            curEvent.transform.position = eventData.position;
            curEvent.GetComponent<ToolTip>().ToolTipText = curEvent.name;
        }
    }

    /// <summary>
    /// Creates a visible marker for the event (with a name if there is one for it)
    /// </summary>
    /// <para><paramref name="curEvent"/> = The event to visualize</para>
    private void visualizeEvent(robotEvent curEvent)
    {
        var eventVis = Instantiate(eventVisualizer, eventHolder.transform);
        eventVis.transform.position = curEvent.position;
        eventVis.GetComponent<ToolTip>().ToolTipText = curEvent.eventName;
        var gripperAtEvent = Instantiate(gripperEvent, curEvent.position, curEvent.rotationGripper);
        gripperAtEvent.transform.parent = eventVis.transform;

    }

    /// <summary>
    /// Adds an event to the list of events and visualizes it directly
    /// </summary>
    /// <para><paramref name="name"/> = The name of the event</para>
    /// <para><paramref name="pos"/> = The position of the gripper at the event</para>
    /// <para><paramref name="rot"/> = The rotation of the gripper at the event</para>

    private void addEvent(string name, Vector3 pos, Quaternion rot)
    {
        robotEvent eventData = new robotEvent { eventName = name, position = pos, rotationGripper = rot };
        recordedEvents.Add(eventData);
        visualizeEvent(eventData);
    }



    /// <summary>
    /// Converts a path to a spline which can then be manually edited
    /// </summary>
    private void convertToSpline()
    {

        if (spline != null && recordedPath.Count / intLen > 2)
        {
            int lastIndex = 0;
            int counterIndexEvents = 0;
            for (int i = 0; i < recordedPath.Count / intLen; i++)
            {
                var splinePoint = spline.InsertNewPointAt(i);
                var pointTransform = splinePoint.transform;

                splinePoint.transform.SetParent(splinePointsHolder.transform);

                if (counterIndexEvents < indexEvents.Count)
                {
                    if (i * intLen < indexEvents[counterIndexEvents])
                    {
                        splinePoint.position = recordedPath[intLen * i];
                        controlPointsIndices.Add(i * intLen);
                    }
                    else // makes sure that events are a controlpoint (continues then from there on with normal interval)
                    {
                        splinePoint.position = recordedPath[indexEvents[counterIndexEvents]];
                        controlPointsIndices.Add(indexEvents[counterIndexEvents]);
                        eventHolder.transform.GetChild(0).parent = splinePoint.transform;
                        counterIndexEvents++;
                    }
                }
                else
                {
                    splinePoint.position = recordedPath[intLen * i];
                    controlPointsIndices.Add(i * intLen);
                }
                splinePoint.AddComponent<NearInteractionGrabbable>();
                splinePoint.AddComponent<ObjectManipulator>();
                splinePoint.GetComponent<Renderer>().material = materialUncheckedControlpoint;
                lastIndex = i;
            }

            #region endpoint is controlpoint
            //make sure endpoint is a control point
            var splinePointEnd = spline.InsertNewPointAt(lastIndex + 1);
            splinePointEnd.transform.SetParent(splinePointsHolder.transform);
            splinePointEnd.position = recordedPath[recordedPath.Count - 1];
            controlPointsIndices.Add(recordedPath.Count - 1);
            splinePointEnd.AddComponent<NearInteractionGrabbable>();
            splinePointEnd.AddComponent<ObjectManipulator>();
            splinePointEnd.GetComponent<Renderer>().material = materialUncheckedControlpoint;
            #endregion

            spline.Refresh();
            spline.AutoConstructSpline();
            spline.drawGizmos = true;
            spline.gizmoSmoothness = 30;
        }
    }

    /// <summary>
    /// Makes spline automaticaly nice when editing it through the controlpoints
    /// </summary>
    private void splineEditing()
    {
        if (spline != null && pointToPlot != null && pointToPlot.Count() > 2)
        {
            spline.AutoConstructSpline();
        }

    }

    /// <summary>
    /// Saves the events back after their positions and maybe also names were changed in the splin editing phase
    /// </summary>
    private void saveEvents()
    {
        var eventToSave = splinePointsHolder.GetComponentsInChildren<ToolTip>();
        foreach (var myEvent in eventToSave)
        {
            robotEvent eventData = new robotEvent { eventName = myEvent.ToolTipText, position = myEvent.transform.position, rotationGripper = myEvent.GetComponentInChildren<ConstraintManager>().gameObject.transform.rotation };
            savedEvents.Add(eventData);
        }
    }


    /// <summary>
    /// Converts the spline back into a path 
    /// </summary>
    private void convertToTrack()
    {
        if (spline != null && spline.Count > 2)
        {
            float fractionAlongSpline = 0f;
            float intervalLength = 1f / recordedPath.Count;
            for (int i = 0; i < recordedPath.Count; i++)
            {

                Vector3 pos = spline.GetPoint(fractionAlongSpline);
                generatedPointsFromSpline.Add(pos);
                fractionAlongSpline = (i + 1) * intervalLength;
            }
        }
        else // spline was not generated --> copy points directly
        {
            generatedPointsFromSpline = recordedPath;
        }

    }

    /// <summary>
    /// Deletes the spline (and controlpoints) and makes it ready for a next one
    /// </summary>
    private void cleanUpSpline()
    {
        controlPointsIndices.Clear();
        if (numberOfTracksSinceStart != 0)
        {
            spline.Initialize(2);
            var BezP = trackObj.transform.GetComponentsInChildren<BezierPoint>();
            foreach (BezierPoint p in BezP)
            {
                if (p.isActiveAndEnabled)
                {
                    p.gameObject.SetActive(false);
                }
            }
        }
    }


    /// <summary>
    /// Delete all events which were visualized and make it ready for a new tracking session
    /// </summary>
    private void cleanUpEvents()
    {
        objectPickedUp = false; //makes sure that always starts with empty gripper, maybe later let user decide
        var eventN = eventHolder.transform.GetComponentsInChildren<ToolTip>();
        foreach (ToolTip eventObj in eventN)
        {
            Destroy(eventObj.gameObject);
        }
        recordedEvents.Clear();
        indexEvents = new List<int>();

    }

    /// <summary>
    /// Shows the current recorded path, also as a liveview.
    /// </summary>
    /// <para><paramref name="state"/> = True to show it and false to hide it</para>
    private void visabilityRecordedTrack(bool state)
    {
        if (trackObj != null)
        {
            var visibility = trackObj.GetComponentsInChildren<LineRenderer>();
            foreach (LineRenderer l in visibility)
            {
                l.GetComponent<LineRenderer>().enabled = state;
            }
        }
    }





    /// <summary>
    ///  Tracks the position of a moving object, used at the moment to track the position and orientation of the gripper attached at the hand.
    /// </summary>
    /// <para><paramref name="onPlane"/> = True to snap the object to a plane and keep it there, nor used at the moment</para>
    private void trackingPos(bool onPlane)
    {
        generatedPointsFromSpline = new List<Vector3>();
        savedEvents = new List<robotEvent>();

        if (firstTrack) //only excecuted once per tracking session
        {

            if (onPlane)
            {
                stayOnPlane(true);
            }
            recordedPath = new List<Vector3>();
            recordedOrientation = new List<Quaternion>();
            recordedOrientationForReattaching = new List<Quaternion>();
            startPosition = trackingPoint.transform.position;
            recordedPath.Add(startPosition);
            recordedOrientation.Add(gripperOrientation.transform.rotation);
            recordedOrientationForReattaching.Add(gripperOrientationForReattaching.transform.rotation);
            lastPosition = startPosition;
            timeLastPointAdded = Time.realtimeSinceStartup;
            firstTrack = false;

        }
        //without onPlane
        if (onPlane == false)
        {
            currentPosition = trackingPoint.transform.position;
            addPointsToPath(currentPosition);
            lastPosition = currentPosition;
        }
        // with onPlane
        else
        {
            stayOnPlane(false);
            currentPosition = trackingPoint.transform.position;
            addPointsToPath(currentPosition);

        }
        showTrack(); // enables live view of the current path


    }




    /// <summary>
    /// Snaps the gameobject which is the tracking point to the plane present in the scene and keeps it there.
    /// </summary>
    /// <para><paramref name="firstTime"/> = Boolean set to true if the snapping has to happen, otherwise if only keep there then false</para>
    private void stayOnPlane(bool firstTime)
    {


        LayerMask layerMask = LayerMask.GetMask("Plane");
        Ray ray = new Ray(trackingPoint.transform.position, -currentPlane.transform.up);
        if (Physics.Raycast(ray, out RaycastHit hit, Mathf.Infinity, layerMask))
        {
            if (Vector3.Magnitude(hit.point - lastPosition) < glitchDistance && firstTime == false)
            {
                trackingPoint.transform.position = hit.point + currentPlane.transform.up * offset;
                Debug.Log(hit.point);
            }
            else if (firstTime)
            {
                trackingPoint.transform.position = hit.point + currentPlane.transform.up * offset;
                Debug.Log(hit.point);
            }

        }
        else // if below plane
        {
            ray = new Ray(trackingPoint.transform.position, currentPlane.transform.up);
            if (Vector3.Magnitude(hit.point - lastPosition) < glitchDistance && firstTime == false)
            {
                trackingPoint.transform.position = hit.point - currentPlane.transform.up * offset;
                Debug.Log(hit.point);
            }
            else if (firstTime)
            {
                trackingPoint.transform.position = hit.point - currentPlane.transform.up * offset;
                Debug.Log(hit.point);
            }
        }
    }


    /// <summary>
    /// Records a point (position and orientation of the gripper) according to the choosen mode as well as handles the recording of events.
    /// </summary>
    /// <para><paramref name="curPos"/> = The position which should be recorded</para>
    private void addPointsToPath(Vector3 curPos)
    {
        #region Position and Orientation
        if (modeTracking == 0) // fixed distance
        {
            while (Vector3.Magnitude(curPos - lastPosition) > trackingRadius)
            {
                Vector3 pointToAdd = Vector3.MoveTowards(lastPosition, curPos, trackingRadius);
                recordedPath.Add(pointToAdd);
                lastPosition = pointToAdd;

            }
            recordedOrientation.Add(gripperOrientation.transform.rotation);
            recordedOrientationForReattaching.Add(gripperOrientationForReattaching.transform.rotation);

        }
        else if (modeTracking == 1 && Time.realtimeSinceStartup - timeLastPointAdded > timeBetweenPointsAdded)
        {
            recordedPath.Add(curPos);
            recordedOrientation.Add(gripperOrientation.transform.rotation);
            recordedOrientationForReattaching.Add(gripperOrientationForReattaching.transform.rotation);
            lastPosition = curPos;
            timeLastPointAdded = Time.realtimeSinceStartup;
        }

        #endregion
        else if (modeTracking != 0 || modeTracking != 1)
        {
            //Debug.LogError("Impossible modeTracking");
        }

        #region adding the events
        if (handPoseDetection.isPinchingTrue && objectPickedUp == false)
        {
            addEvent("Pick-Up", lastPosition, gripperAtHand.transform.rotation);
            objectPickedUp = true;
            indexEvents.Add(recordedPath.Count - 1);
            return; // otherwise immediately place also happens

        }
        else if (handPoseDetection.isPinchingTrue && objectPickedUp)
        {
            addEvent("Place", lastPosition, gripperAtHand.transform.rotation);
            objectPickedUp = false;
            indexEvents.Add(recordedPath.Count - 1);
        }
        else if (handPoseDetection.isVictoryTrue)
        {
            addEvent("Empty", lastPosition, gripperAtHand.transform.rotation);
            indexEvents.Add(recordedPath.Count - 1);
        }
        #endregion


    }


    /// <summary>
    /// Handles what all needs to happen when a new recoding of a path starts
    /// </summary>
    public void enableTracking()
    {
        showSavedWorkflow(false);
        if(!toggleVisabilityWorkflow)
        {
            toggleVisabilityWorkflow = true;
        }
        setGripperToHandSkript.firstAttachGripper = true;
        isClickable(buttonsNotClickableWhileTracking, false);
        cleanUpEvents();
        showLastTrack = false;
        trackingNow = true;
        firstTrack = true;

        if (displayedPath != null)
        {
            displayedPath.positionCount = 0;
        }
        trackObj.SetActive(true);
        cleanUpSpline();
        convertToSplineHappened = false;
        numberOfTracksSinceStart++;
        StartCoroutine(destoryAllChildren(checkedPositionsHolder));
        visabilityRecordedTrack(true);
        indexLastChecked = 0;
        indexLastSuccesfullCheck = 0;
        recordingActive = true;
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


    /// <summary>
    /// Handles what needs to happen if tracking should happen on a plane, afterwards normal tracking happens
    /// </summary>
    public void enableTrackingOnPlane()
    {

        trackOnPlane = true;
        var curPlane = currentPlane.transform;
        enableTracking();
    }

    /// <summary>
    /// Handles what needs to happen after the tracking should stop
    /// </summary>
    public void disableTracking()
    {
        isClickable(buttonsNotClickableWhileTracking, true);
        gripperAtHand.SetActive(false);
        trackingNow = false;
        firstTrack = false;
        trackOnPlane = false;
        recordingActive = false;

        if (modeTracking == 1)
        {
            checkROS(recordedPath);
            #region old ROS check        
            /*
            if (checkROS(recordedPath))
            {
                indexLastChecked = recordedPath.Count - 1;
                Debug.Log("lastIndex checked is " + indexLastChecked);
            }
            else
            {
                recordedPath.RemoveRange(indexLastChecked + 1, recordedPath.Count - indexLastChecked - 1);
                recordedOrientation.RemoveRange(indexLastChecked + 1, recordedPath.Count - indexLastChecked - 1);
                setGripperToHandSkript.lastVisibleGripperPosition = recordedPath[indexLastChecked];
                setGripperToHandSkript.setLastValidGripper(recordedPath[indexLastChecked], recordedOrientation[indexLastChecked]);
                setGripperToHandSkript.firstAttachGripper = false;
            }
        */
            #endregion

        }
        showTrack();
    }

    /// <summary>
    /// Handles what needs to happen to generate the spline
    /// </summary>
    public void enableSpline()
    {
        showLastTrack = true;
        if (numberOfTracksSinceStart != 0 && convertToSplineHappened == false)
        {
            convertToSpline();
            convertToSplineHappened = true;
        }



    }


    /// <summary>
    /// Converts the current spline back into a simple path and saves it as well as hides the path and spline
    /// </summary>
    public void disableShowAndSave()
    {
        saveEvents();
        convertToTrack();
        showLastTrack = false;
        visabilityRecordedTrack(false);
        cleanUpSpline();
        cleanUpEvents();
        StartCoroutine(destoryAllChildren(checkedPositionsHolder));
        createSnippetAndAddToWorkflow(addSnippetAtEnd);
    }


    /// <summary>
    /// Shows the path whhich was recorded so far
    /// </summary>
    private void showTrack()
    {
        if (recordedPath.Count != 0)
        {
            displayedPath = trackObjRecorded.GetComponent<LineRenderer>();
            pointToPlot = new Vector3[recordedPath.Count];
            displayedPath.positionCount = recordedPath.Count;
            for (int i = 0; i < recordedPath.Count; i++)
            {
                pointToPlot[i] = recordedPath[i];
            }
            displayedPath.SetPositions(pointToPlot);
        }
    }

    /// <summary>
    /// Changes the visability of the saved total workflow
    /// </summary>
    public void toggleShowSavedWorkflow()
    {
        showSavedWorkflow(toggleVisabilityWorkflow);
        toggleVisabilityWorkflow = !toggleVisabilityWorkflow;

    }


    /// <summary>
    /// Visualizes the total saved workflow (only path) with the end- and startpoint
    /// <para><paramref name="state"/> = True if should be created and shown, false if should hide </para>
    /// </summary>
    public void showSavedWorkflow(bool state)
    {
        if (state)
        {
            if (savedWorkflow.Count != 0)
            {
                savedWorkflowPath.gameObject.SetActive(true);
                List<Vector3> savedPoints = new List<Vector3>();
                foreach (var snippet in savedWorkflow)
                {
                    foreach (var pos in snippet.position)
                    {
                        savedPoints.Add(pos);
                    }

                }
                savedPos = savedPoints.ToArray();
                savedWorkflowPath.positionCount = savedPos.Length;
                savedWorkflowPath.SetPositions(savedPos);
                startPointWorkflow.transform.position = savedPos[0];
                endPointWorkflow.transform.position = savedPos[savedPos.Length - 1];
                startPointWorkflow.SetActive(true);
                endPointWorkflow.SetActive(true);
                endPointWorkflow.GetComponent<Renderer>().material = materialEnd;
                startPointWorkflow.GetComponent<Renderer>().material = materialStart;
                endPointWorkflow.transform.localScale = new Vector3(baseRadiusActivateNear, baseRadiusActivateNear, baseRadiusActivateNear);
                startPointWorkflow.transform.localScale = new Vector3(baseRadiusActivateNear, baseRadiusActivateNear, baseRadiusActivateNear);
            }
        }
        else
        {
            savedWorkflowPath.gameObject.SetActive(false);
        }


    }

    /// <summary>
    /// Creates a snippet out of the just recorded path and events and adds it to the current worflow either at the end or beginning.
    /// <para><paramref name="end"/> = Set to true if snippet should be added at the end, false if should be added at the beginning </para>
    /// </summary>
    private void createSnippetAndAddToWorkflow(bool end)
    {
        if (end)
        {
            var defaultOrientation = new List<Quaternion>();
            defaultOrientation.Add(recordedOrientation[0]);
            defaultOrientation.Add(recordedOrientation[recordedOrientation.Count - 1]);

            var defaultOrientationForReatttaching = new List<Quaternion>();
            defaultOrientationForReatttaching.Add(recordedOrientationForReattaching[0]);
            defaultOrientationForReatttaching.Add(recordedOrientationForReattaching[recordedOrientationForReattaching.Count - 1]);

            var curSnippet = new savedSnippets { position = generatedPointsFromSpline, mode = modeTracking, orientation = defaultOrientation, events = savedEvents, orientationForReattaching = defaultOrientationForReatttaching };
            if (modeTracking == 1)
            {
                curSnippet.orientation = recordedOrientation;
                curSnippet.orientationForReattaching = recordedOrientationForReattaching;
            }
            savedWorkflow.Add(curSnippet);
        }
        else
        {
            var tempPos = generatedPointsFromSpline.ToArray().Reverse().ToList();
            var tempRot = recordedOrientation.ToArray().Reverse().ToList();
            var tempRotReattach = recordedOrientationForReattaching.ToArray().Reverse().ToList();
            var defaultOrientation = new List<Quaternion>();
            defaultOrientation.Add(Quaternion.identity);
            defaultOrientation.Add(Quaternion.identity);

            var defaultOrientationForReattaching = new List<Quaternion>();
            defaultOrientationForReattaching.Add(Quaternion.identity);
            defaultOrientationForReattaching.Add(Quaternion.identity);


            var curSnippet = new savedSnippets { position = tempPos, mode = modeTracking, orientation = defaultOrientation, events = savedEvents, orientationForReattaching = defaultOrientationForReattaching };
            if (modeTracking == 1)
            {
                curSnippet.orientation = tempRot;
                curSnippet.orientationForReattaching = tempRotReattach;
            }
            savedWorkflow.Insert(0, curSnippet);
        }

    }



    /// <summary>
    /// Starts the new recording at the selected point
    /// <para><paramref name="state"/> = True if at the end, false for the beginning </para>
    /// </summary>
    public void setWhereToAddSnippet(bool state)
    {
        addSnippetAtEnd = state;
        if (state)
        {
            endPointWorkflow.GetComponent<Renderer>().material = materialCurrent;
            startPointWorkflow.GetComponent<Renderer>().material = materialStart;

            positionContinueTracking = endPointWorkflow.transform.position;
            rotationContinueTracking = savedWorkflow.Last<savedSnippets>().orientationForReattaching[savedWorkflow.Last<savedSnippets>().orientationForReattaching.Count - 1];
            startPointWorkflow.transform.localScale = new Vector3(baseRadiusActivateNear, baseRadiusActivateNear, baseRadiusActivateNear);
            endPointWorkflow.transform.localScale = new Vector3(radiusActivateNearPoint, radiusActivateNearPoint, radiusActivateNearPoint);
        }
        else
        {
            startPointWorkflow.GetComponent<Renderer>().material = materialCurrent;
            endPointWorkflow.GetComponent<Renderer>().material = materialEnd;
            positionContinueTracking = startPointWorkflow.transform.position;
            rotationContinueTracking = savedWorkflow.First<savedSnippets>().orientationForReattaching[0];
            endPointWorkflow.transform.localScale = new Vector3(baseRadiusActivateNear, baseRadiusActivateNear, baseRadiusActivateNear);
            startPointWorkflow.transform.localScale = new Vector3(radiusActivateNearPoint, radiusActivateNearPoint, radiusActivateNearPoint);
        }
        reattachedAfterContinue = false;
    }



    /// <summary>
    /// Initializes the check if the spline is reachable (for this creates a path out of the spline first if needed)
    /// </summary>
    public void initializeCheckReachableSpline()
    {
        if (spline != null && spline.Count > 2) // spline was generated
        {

            var controlPoints = splinePointsHolder.GetComponentsInChildren<BezierPoint>();
            int indexControlPoint = 1;
            controlPointsIndices.Clear();
            controlPointsIndices.Add(0);

            //pointsToCheck.Clear();
            pointsToCheck = new List<Vector3>();
            float fractionAlongSpline = 0f;
            float intervalLength = 1f / recordedPath.Count;
            for (int i = 0; i < recordedPath.Count; i++)
            {
                Vector3 pos = spline.GetPoint(fractionAlongSpline);
                pointsToCheck.Add(pos);
                fractionAlongSpline = (i + 1) * intervalLength;
                if (indexControlPoint < controlPoints.Length - 1 && Vector3.Distance(pos, controlPoints[indexControlPoint].position) < 0.005)
                {
                    controlPointsIndices.Add(i);
                    indexControlPoint++;
                }

            }
            controlPointsIndices.Add(recordedPath.Count - 1);




        }
        checkROS(pointsToCheck);
    }

    


    /// <summary>
    /// Shows if the path is reachable or which parts are not by coloring the controlpoints of the nonreachable part red and the reachable ones green. 
    /// Call this one in the callback function of the subscribe to the result of full path check from ROS
    /// </summary>
    /// <para><paramref name="nonReachablePositionIndices"/> = An array consisting of pairs of start and end-index of the nonreachable parts of the path e.g [4,8, 12,15] for the nonreachability of 4-8 and 12-15 </para>
    /// <returns> True if the whole path is reachable, otherwise false</returns>
    public bool showResultReachabilityPath(int[] nonReachablePositionIndices)
    {
        if (nonReachablePositionIndices.Length == 0)
        {

            //color all points green
            var controlPoints = splinePointsHolder.GetComponentsInChildren<BezierPoint>();
            foreach (var point in controlPoints)
            {
                point.GetComponent<Renderer>().material = materialCorrectControlpoint;
            }
            return true;
            //Maybe do more stuff here, let user know that successfull and let him save the path
        }
        else
        {
            // color the error ones red
            visualizeNonReachableEquidistant(nonReachablePositionIndices);
            return false;
        }

    }

    /// <summary>
    /// Initializes the check with ROS depending on the current mode to check if the robot can reach every position
    /// </summary>
    /// <returns>True if succesfull, otherwise false (if wrong mode) </returns>
    public bool checkROS(List<Vector3> checkPath)
    {
        if (modeTracking == 0)
        {
            ROSConnectionScript.checkIfPathIsReachableROS(checkPath);
            return true;
        }

        else if (modeTracking == 1)
        {
            ROSConnectionScript.checkCurrentPositionReachableROS(checkPath.Last(), recordedOrientation.Last());
            indexLastChecked = checkPath.Count -1;
            return true;
            #region old version of the check
            /* old version of the check
            bool isValid = ROSConnectionScript.isPointReachable;


            if (isValid)
            {
                Debug.Log("Time Mode point is reachable");
                lastValidPosition = checkPath.Last();
                var validPoint = Instantiate(markerValidPosition, lastValidPosition, Quaternion.identity);
                validPoint.transform.parent = checkedPositionsHolder.transform;
                return true;
            }
            else
            {
                ROSConnectionScript.isPointReachable = true;
                return false;
            }
            */
            #endregion
        }
        else
        {
            Debug.LogError("Wrong mode for ROS check");
            return false;
        }
    }


    /// <summary>
    /// Function to handle the result of the check of a single point while tracking (time mode), basically removes the unreachable part of the recorded path.
    /// </summary>
    /// <para><paramref name="isValid"/> = True if the last part was all reachable, false otherwise </para>
    public void visualizeReachabilityTime(bool isValid)
    {
        if (isValid)
        {
            lastValidPosition = recordedPath[indexLastChecked];
            indexLastSuccesfullCheck = indexLastChecked;
            var validPoint = Instantiate(markerValidPosition, lastValidPosition, Quaternion.identity);
            validPoint.transform.parent = checkedPositionsHolder.transform;
        }
        else
        {
            recordedPath.RemoveRange(indexLastSuccesfullCheck + 1, recordedPath.Count -indexLastSuccesfullCheck- 1);
            recordedOrientation.RemoveRange(indexLastSuccesfullCheck + 1, recordedOrientation.Count - indexLastSuccesfullCheck - 1);
            recordedOrientationForReattaching.RemoveRange(indexLastSuccesfullCheck + 1, recordedOrientationForReattaching.Count - indexLastSuccesfullCheck - 1);

            for (int i = 0; i < indexEvents.Count; i++)
            {
                if (indexEvents[i] > indexLastSuccesfullCheck)
                {
                    indexEvents.RemoveAt(i);
                    recordedEvents.RemoveAt(i);
                    i--;
                    Destroy(eventHolder.transform.GetChild(eventHolder.transform.childCount - 1).gameObject);
                }
            }
            setGripperToHandSkript.lastVisibleGripperPosition = recordedPath[indexLastSuccesfullCheck];
            gripperAtHand.SetActive(false);
            setGripperToHandSkript.setLastValidGripper(recordedPath[indexLastSuccesfullCheck], recordedOrientationForReattaching[indexLastSuccesfullCheck]);
            setGripperToHandSkript.firstAttachGripper = false;
        }
    }


    /// <summary>
    /// Visualizes the non-reachable parts of the spline in the equidistance mode, checkout showResultReachabilityPath to see where this function is needed.
    /// </summary>
    /// <para><paramref name="nonReachablePositionIndices"/> = An array consisting of pairs of start and end-index of the nonreachable parts of the path e.g [4,8, 12,15] for the nonreachability of 4-8 and 12-15 </para>
    private void visualizeNonReachableEquidistant(int[] nonReach)
    {
        //show the last reachable position
        moveRobotToPosition.visualizeLastPossibleRobotPosition();
        int[] indexPositionForNearestControlPoint = new int[nonReach.Length];
        for (int i = 0; i < nonReach.Length; i++)
        {
            indexPositionForNearestControlPoint[i] = controlPointsIndices.OrderBy(item => Math.Abs(nonReach[i] - item)).First();
        }

        var controlPoints = splinePointsHolder.GetComponentsInChildren<BezierPoint>();
        bool inErrorRange = false;
      



        int counterUnreachble = 0;
        foreach (var point in controlPoints)
        {
            point.GetComponent<Renderer>().material = materialCorrectControlpoint; //color all points green
            //color the ones in error range red
            if (Vector3.Distance( point.gameObject.transform.position , pointsToCheck[indexPositionForNearestControlPoint[counterUnreachble]]) < distanceCP)
            {
                inErrorRange = true;
                point.GetComponent<Renderer>().material = materialErrorControlpoint;

            }
            else if (inErrorRange)
            {
                point.GetComponent<Renderer>().material = materialErrorControlpoint;
            }
            if (Vector3.Distance( point.gameObject.transform.position , pointsToCheck[indexPositionForNearestControlPoint[counterUnreachble + 1]]) < distanceCP)
            {
                inErrorRange = false;
                if (counterUnreachble + 2 < indexPositionForNearestControlPoint.Length)
                {
                    counterUnreachble += 2;
                }
            }
        }
    }


    /// <summary>
    /// Deletes the last recorded path and makes it ready for a new recording
    /// </summary>
    public void deleteTrack()
    {
        showLastTrack = false;
        visabilityRecordedTrack(false);
        cleanUpSpline();
        displayedPath = null;
        recordedPath = new List<Vector3>();
        pointToPlot = null;
        cleanUpEvents();
        StartCoroutine(destoryAllChildren(checkedPositionsHolder));
    }
}
