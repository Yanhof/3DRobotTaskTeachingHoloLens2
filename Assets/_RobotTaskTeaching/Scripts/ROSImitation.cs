using System.Collections.Generic;
using UnityEngine;


/// <summary>
/// This script allows a random check concerning reachability and mimicks therefore the ROS connection
/// </summary>
public class ROSImitation : MonoBehaviour
{
    int howOftenReachable = 5; //10 always, 0 never



    /// <summary>
    /// Random check if path is reachable
    /// <para><paramref name="pathToCheck"/> = The path to check (positions) </para>
    /// <returns> -1 if reachable, otherwise the indices where not reachable begins and ends </returns>
    ///  </summary>
    public int[] checkIfPathIsReachable(List<Vector3> pathToCheck)
    {
        if(UnityEngine.Random.Range(0, 10) < howOftenReachable) {
            return new int[] { -1 };
        }
        else
        {
                int lower = UnityEngine.Random.Range(0, pathToCheck.Count - 2);
                int higher = UnityEngine.Random.Range(lower+50, pathToCheck.Count - 1);
                return new int[] {lower, lower +50, higher -30, higher};
        }
       
    }

    /// <summary>
    /// Random check if pose is reachable
    /// <para><paramref name="posToCheck"/> = The position to check</para>
    /// <para><paramref name="orientationGripper"/> = The orientation of the end effector to check </para>
    /// <returns> True if reachable, otherwise false </returns>
    ///  </summary>
    public bool checkCurrentPositionReachable(Vector3 posToCheck, Quaternion orientationGripper)
    {
        if (UnityEngine.Random.Range(0, 10) < howOftenReachable)
        {
            return true;
        }
        else
        {
            return false;
        }

    }
}
