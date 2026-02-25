using System.IO;
using UnityEngine;
using UnityEngine.Networking;
using System.Collections;

#if UNITY_EDITOR
using UnityEditor;
#endif

[System.Serializable]
public class DirectionResponse
{
    public string direction;
}

public class StereoCaptureSaver : MonoBehaviour
{
    [Header("Stereo Cameras")]
    public Camera leftCam;
    public Camera rightCam;

    [Header("Render Texture Settings")]
    public int widthPerEye = 1024;
    public int height = 768;
    public float baseline = 0.06f;

    private RenderTexture leftRT;
    private RenderTexture rightRT;

    void Start()
    {
        if (leftCam == null || rightCam == null)
        {
            Debug.LogError("Left veya Right kamera atanmadı.");
            enabled = false;
            return;
        }

        leftCam.transform.localPosition = Vector3.left * (baseline / 2f);
        rightCam.transform.localPosition = Vector3.right * (baseline / 2f);

        leftRT = new RenderTexture(widthPerEye, height, 24, RenderTextureFormat.ARGB32);
        rightRT = new RenderTexture(widthPerEye, height, 24, RenderTextureFormat.ARGB32);

        leftRT.Create();
        rightRT.Create();

        leftCam.targetTexture = leftRT;
        rightCam.targetTexture = rightRT;
    }

    void Update()
    {
        if (Input.GetKeyDown(KeyCode.X))
        {
            CaptureStereoAndSend();
        }
    }

    void CaptureStereoAndSend()
    {
        leftCam.Render();
        rightCam.Render();

        Texture2D leftTex = new Texture2D(widthPerEye, height, TextureFormat.RGB24, false);
        Texture2D rightTex = new Texture2D(widthPerEye, height, TextureFormat.RGB24, false);

        RenderTexture prev = RenderTexture.active;

        RenderTexture.active = leftRT;
        leftTex.ReadPixels(new Rect(0, 0, widthPerEye, height), 0, 0);
        leftTex.Apply();

        RenderTexture.active = rightRT;
        rightTex.ReadPixels(new Rect(0, 0, widthPerEye, height), 0, 0);
        rightTex.Apply();

        RenderTexture.active = prev;

#if UNITY_EDITOR
        SaveStereoForDebug(leftTex, rightTex);
#endif

        StartCoroutine(SendToPythonAPI(leftTex, rightTex));

        Destroy(leftTex);
        Destroy(rightTex);
    }

    IEnumerator SendToPythonAPI(Texture2D leftTex, Texture2D rightTex)
    {
        byte[] leftBytes = leftTex.EncodeToPNG();
        byte[] rightBytes = rightTex.EncodeToPNG();

        WWWForm form = new WWWForm();
        form.AddBinaryData("left_image", leftBytes, "left.png", "image/png");
        form.AddBinaryData("right_image", rightBytes, "right.png", "image/png");

        using (UnityWebRequest www =
            UnityWebRequest.Post("http://127.0.0.1:5000/upload_stereo", form))
        {
            yield return www.SendWebRequest();

            if (www.result != UnityWebRequest.Result.Success)
            {
                Debug.LogError("API Error: " + www.error);
            }
            else
            {
                DirectionResponse response =
                    JsonUtility.FromJson<DirectionResponse>(www.downloadHandler.text);

                Debug.Log("Stereo navigation suggestion: " + response.direction);
            }
        }
    }

#if UNITY_EDITOR
    void SaveStereoForDebug(Texture2D leftTex, Texture2D rightTex)
    {
        Texture2D combined =
            new Texture2D(widthPerEye * 2, height, TextureFormat.RGB24, false);

        for (int y = 0; y < height; y++)
        {
            combined.SetPixels(0, y, widthPerEye, 1,
                leftTex.GetPixels(0, y, widthPerEye, 1));
            combined.SetPixels(widthPerEye, y, widthPerEye, 1,
                rightTex.GetPixels(0, y, widthPerEye, 1));
        }

        combined.Apply();

        string folder = Path.Combine(Application.dataPath, "Images");
        if (!Directory.Exists(folder)) Directory.CreateDirectory(folder);

        string fname = $"stereo_{System.DateTime.Now:HHmmssfff}.png";
        File.WriteAllBytes(Path.Combine(folder, fname), combined.EncodeToPNG());

        AssetDatabase.Refresh();
        Destroy(combined);
    }
#endif

    void OnDestroy()
    {
        if (leftRT != null) leftRT.Release();
        if (rightRT != null) rightRT.Release();
    }
}
