using UnityEngine;
using UnityEngine.SceneManagement;

public class BA_SceneSelector : MonoBehaviour {

    public void returnToSelector() {
        SceneManager.LoadScene("BachelorThesisSelector");
    }
    public void switchToTPI() {
        SceneManager.LoadScene("TaskPlanningInterface");
    }

    public void switchToAsync() {
        SceneManager.LoadScene("AsynchronousRobotTeaching");
    }

    public void reloadScene() {
        SceneManager.LoadScene(SceneManager.GetActiveScene().name);
    }

}
