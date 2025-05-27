#if UNITY_EDITOR
using System;
using UnityEditor;
using UnityEngine;
using UnityEngine.UIElements;
using UnityEditor.UIElements;
using UnityEngine.Networking;
using System.Collections.Generic;
using GLTFast.Export;
using GLTFast;
using System.IO;
using System.Collections;
using UnityEngine.PlayerLoop;

/// <summary>
/// A window can render many GUIModel.
/// One GUIModel bear one method.
/// Include sending web request, getting response, getting the information you want to send, rendering the information you want to show.
/// If you want to have more than one kind of web requests, please finish your own function to select witch request you want to send.  
/// 
/// How to use:
/// 1. Create a class witch is inheritance of GUIModel.
/// 2. Override the functions, witch you need.
/// 3. You have to override Draw().
/// 4. Use the Draw function in OnGUI function.
/// </summary>
public class GUIModel
{
    static public string API_KEY_FIELD = "Meshy API Keys";
    public class jsonObj
    {
        public string message;
    }

    // Here achieve the web request.
    public virtual void WebRequest()
    {
        Debug.Log("Here override the WebRequest function.");
    }

    // Here achieve the GUI rendering.
    public virtual void Draw()
    {
        Debug.Log("Here override the Draw function.");
    }

    public void CheckErrorCode(long errorCode,string text)
    {
        switch (errorCode)
        {
            case 400:
            {
                Debug.LogError("Bad Request!");
                break;
            }
            case 401:
            {
                Debug.LogError("Unauthorized!");
                break;
            }
            case 402:
            {
                Debug.LogError("Payment Required!");
                break;
            }
            case 404:
            {
                Debug.LogError("Not Found!");
                break;
            }
            case 429:
            {
                Debug.LogError("Too Many Requests!");
                break;
            }
            case var n when (n>=500&&n<600):
            {
                Debug.LogError("Server Error!");
                break;
            }
            default:
            {
                Debug.LogError("Unknown Error.");
                break;
            }
        }
        jsonObj jobj;
        jobj = JsonUtility.FromJson<jsonObj>(text);
        Debug.LogError(jobj.message);
    }
}

/// <summary>
/// This model is used to submit the information of local mesh.
/// For text to texture.
/// </summary>
public class SubmitMeshInfo : GUIModel
{
    private string[] atrStyleOptions = new string[]
    {
        "Realistic",
        "2.5D Cartoon",
        "Cartoon Line Art",
        "2.5D Hand-drawn",
        "Japanese Anime",
        "Realistic Hand-drawn",
        "Oriental Comic Lnk"
    };

    private string[] atrStyleValue = new string[]
    {
        "realistic",
        "fake-3d-cartoon",
        "cartoon-line-art",
        "fake-3d-hand-drawn",
        "japanese-anime",
        "realistic-hand-drawn",
        "oriental-comic-ink"
    };

    private string objectPrompt;
    private string stylePrompt;
    private string negativePrompt;
    private int artStyle;
    private bool enableOriginalUv;
    private bool enablePbr;
    public GameObject selectedObject;
    private string taskName = "Meshy-model";

    static string OBJECT_PROMPT_UI_NAME = "Object Prompt";
    static string STYLE_PROMPT_UI_NAME = "Style Prompt";
    private static string ART_STYLE_UI_NAME = "Art Style";
    static string ENALE_ORIGINAL_UV_UI_NAME = "Enable Orginal UV";
    private static string ENABLE_PBR_UI_NAME = "Enable PBR";
    static string NEGATIVE_PROMPT_UI_NAME = "Negative Prompt";
    static string TASK_NAME_UI_NAME = "Task Name";
    static string TEXT_TO_TEXTURE_URL = "https://api.meshy.ai/v1/text-to-texture";

    async private void ExportGlb()
    {
        // Export glb by glTFast
        // -------------exporting setting-----------------
        var exportSettings = new ExportSettings
        {
            Format = GltfFormat.Binary,
            FileConflictResolution = FileConflictResolution.Overwrite,
            ComponentMask = ~(GLTFast.ComponentType.Camera | GLTFast.ComponentType.Animation | GLTFast.ComponentType.Light),
        };

        var gameObjectExportSettings = new GameObjectExportSettings
        {
            OnlyActiveInHierarchy = false,
            DisabledComponents = true,
        };
        //-------------exporting setting-----------------

        // Export the selected object into binary data
        var export = new GameObjectExport(exportSettings, gameObjectExportSettings);
        GameObject[] exportGameObject = new GameObject[1];
        Vector3 vcache = selectedObject.transform.position;
        selectedObject.transform.position = Vector3.zero;
        exportGameObject[0] = selectedObject;
        export.AddScene(exportGameObject);
        selectedObject.transform.position = vcache;
        Stream stream = new MemoryStream();
        bool success = await export.SaveToStreamAndDispose(stream);
        if (!success)
        {
            Debug.LogError("Converting unity asset to GLB failed.");
            return;
        }
        else
        {
            // Upload data
            byte[] data = new byte[stream.Length];
            stream.Seek(0, SeekOrigin.Begin);
            stream.Read(data, 0, (int)stream.Length);
            WWWForm form = new WWWForm();
            form.AddField("object_prompt", objectPrompt);
            form.AddField("style_prompt", stylePrompt);
            form.AddField("negative_prompt", negativePrompt == null ? "" : negativePrompt);
            form.AddField("enable_original_uv", enableOriginalUv.ToString());
            form.AddField("enable_pbr", enablePbr.ToString());
            form.AddField("art_style", atrStyleValue[artStyle]);
            form.AddField("name", taskName);
            form.AddBinaryData("model_file", data, taskName + ".glb", "multipart/form-data");
            UnityWebRequest request = UnityWebRequest.Post(TEXT_TO_TEXTURE_URL, form);
            request.SetRequestHeader("Authorization", EditorPrefs.GetString(API_KEY_FIELD));
            request.downloadHandler = new DownloadHandlerBuffer();
            request.SendWebRequest().completed += OnRequestCompleted;
        }
    }

    void OnRequestCompleted(AsyncOperation operation)
    {
        UnityWebRequest request = ((UnityWebRequestAsyncOperation)operation).webRequest;
        if (request.result == UnityWebRequest.Result.Success)
        {
            if (request.downloadHandler != null)
            {
                string submittedTaskInfo = request.downloadHandler.text;
                Debug.Log("Submit successfully. Task info: " + submittedTaskInfo);
            }
            else
            {
                Debug.LogError("No response.");
            }
        }
        else
        {
            CheckErrorCode(request.responseCode,request.downloadHandler.text);
        }
    }

    public override void WebRequest()
    {
        ExportGlb();
    }

    public override void Draw()
    {
        objectPrompt = EditorGUILayout.TextField(OBJECT_PROMPT_UI_NAME, objectPrompt);
        stylePrompt = EditorGUILayout.TextField(STYLE_PROMPT_UI_NAME, stylePrompt);
        negativePrompt = EditorGUILayout.TextField(NEGATIVE_PROMPT_UI_NAME, negativePrompt);
        artStyle = EditorGUILayout.Popup(ART_STYLE_UI_NAME, artStyle, atrStyleOptions);
        enableOriginalUv = EditorGUILayout.Toggle(ENALE_ORIGINAL_UV_UI_NAME, enableOriginalUv);
        enablePbr = EditorGUILayout.Toggle(ENABLE_PBR_UI_NAME, enablePbr);
        selectedObject = EditorGUILayout.ObjectField("Selected GameObject", selectedObject, typeof(GameObject), true) as GameObject;
        if (taskName == null || taskName == "")
        {
            taskName = "Meshy-model";
        }
        taskName = EditorGUILayout.TextField(TASK_NAME_UI_NAME, taskName);

        if (GUILayout.Button("Submit Task"))
        {
            if (!EditorPrefs.HasKey(API_KEY_FIELD))
            {
                Debug.LogError("No saved API key found!");
                return;
            }

            if (objectPrompt == null || objectPrompt == "")
            {
                Debug.LogError("Object Prompt can not be empty!");
                return;
            }

            if (stylePrompt == null || stylePrompt == "")
            {
                Debug.LogError("Style Prompt can not be empty!");
                return;
            }

            if (taskName == null || taskName == "")
            {
                Debug.LogError("Task Name can not be empty!");
                return;
            }
            ExportGlb();
        }
    }
}

/// <summary>
/// This model is used to refresh the task list and download the model.
/// For text to texture, text to 3d, image to 3d.
/// </summary>
public class GetTaskList : GUIModel
{
    [System.Serializable]
    public class ReturnedTaskProgress
    {
        public string id;
        public string name;
        public string art_style;
        public string object_prompt;
        public string style_prompt;
        public string negative_prompt;
        public string status;
        public long created_at;
        public int progress;
        public long started_at;
        public long finished_at;
        public long expires_at;
        public string task_error;
        public ModelUrl model_urls;
        public string thumbnail_url;
        public TextureUrl texture_urls;
    }

    [System.Serializable]
    public class ReturnedTasks
    {
        public List<ReturnedTaskProgress> tasks;
    }

    [System.Serializable]
    public class ModelUrl
    {
        public string glb;
        public string fbx;
        public string usdz;
    }

    [System.Serializable]
    public class TextureUrl
    {
        public string base_color;
        public string metallic;
        public string normal;
        public string roughness;
    }

    private ReturnedTasks tasks = null;
    public bool isRefreshing = false;
    private string TEXT_TO_TEXTURE_URL;
    private string fileName;
    private string buttonStr = "Enable Auto Refreshing Task List";

    // The beginning position of the task list's scroll
    private Vector2 scrollPosition = new(0, 0);

    // Start time of timestamp
    private System.DateTime startTime = TimeZoneInfo.ConvertTimeFromUtc(new System.DateTime(1970, 1, 1, 0, 0, 0, DateTimeKind.Utc), TimeZoneInfo.Local);

    /// <summary>
    /// Chose the 
    /// </summary>
    /// <param name="url">The url of text to texture or text to 3d, image to 3d.</param>
    public GetTaskList(string url)
    {
        TEXT_TO_TEXTURE_URL = url;
    }

    public void Refresh()
    {
        // Check the key
        if (!EditorPrefs.HasKey(API_KEY_FIELD))
        {
            Debug.LogError("No saved API key!");
            return;
        }
        // Send request
        UnityWebRequest request = UnityWebRequest.Get(TEXT_TO_TEXTURE_URL + "?sortBy=-created_at");
        request.SetRequestHeader("Authorization", EditorPrefs.GetString(API_KEY_FIELD));
        request.downloadHandler = new DownloadHandlerBuffer();
        request.SendWebRequest().completed += OnGetProgressCompleted;
    }

    void OnGetProgressCompleted(AsyncOperation operation)
    {
        UnityWebRequest request = ((UnityWebRequestAsyncOperation)operation).webRequest;
        if (request.result == UnityWebRequest.Result.Success)
        {
            if (request.downloadHandler != null)
            {
                string json = "{\"tasks\":" + request.downloadHandler.text + "}";
                tasks = JsonUtility.FromJson<ReturnedTasks>(json);
                // Debug.Log("Refresh completed!");
            }
            else
            {
                Debug.LogError("No response.");
            }
        }
        else
        {
            CheckErrorCode(request.responseCode,request.downloadHandler.text);
        }
    }

    void DownloadCompleted(AsyncOperation operation)
    {
        UnityWebRequest request = ((UnityWebRequestAsyncOperation)operation).webRequest;
        if (request.result == UnityWebRequest.Result.Success)
        {
            if (request.downloadHandler != null)
            {
                byte[] results = request.downloadHandler.data;
                string rPath = Application.dataPath;
                if (!File.Exists(rPath + "/Meshy"))
                {
                    Directory.CreateDirectory(rPath + "/Meshy");
                }

                // Set importing path
                string path;
                if (fileName == "" || fileName == null)
                {
                    path = rPath + "/Meshy" + "/" + "Meshy-model.glb";
                    fileName = "Meshy-model";
                }
                path = rPath + "/Meshy" + "/" + fileName + ".glb";

                int index = 0;
                while (File.Exists(path) == true)
                {
                    path = rPath + "/Meshy" + "/" + fileName + index.ToString() + ".glb";
                    index++;
                }

                // Write binary data into file
                File.WriteAllBytes(path, results);
                AssetDatabase.Refresh();
                Debug.Log("Download " + fileName + " completed!");
                request.downloadHandler.Dispose();
            }
            else
            {
                Debug.LogError("No response.");
            }
        }
        else
        {
            CheckErrorCode(request.responseCode,request.downloadHandler.text);
        }
    }

    public override void Draw()
    {
        if (GUILayout.Button(buttonStr))
        {
            isRefreshing = !isRefreshing;
            if(isRefreshing==true)
            {
                buttonStr = "Disable Auto Refreshing Task List";
                Refresh();
            }
            else
            {
                buttonStr = "Enable Auto Refreshing Task List";
            }
        }

        if (tasks != null)
        {
            scrollPosition = GUILayout.BeginScrollView(scrollPosition, false, true);
            GUILayout.BeginVertical(GUILayout.Width(800));

            // Head label
            GUILayout.BeginHorizontal();
            GUILayout.Label("Download", GUILayout.Width(100));
            GUILayout.Label("Task Name", GUILayout.Width(100));
            GUILayout.Label("Art Style", GUILayout.Width(150));
            GUILayout.Label("Progress", GUILayout.Width(80));
            GUILayout.Label("Status", GUILayout.Width(100));
            GUILayout.Label("Create Time", GUILayout.Width(150));
            GUILayout.EndHorizontal();

            // Task list
            for (int i = 0; i < tasks.tasks.Count; i++)
            {
                // Skip expired task
                if (tasks.tasks[i].status == "EXPIRED")
                {
                    continue;
                }
                GUILayout.BeginHorizontal();
                if (tasks.tasks[i].progress == 100)
                {
                    if (GUILayout.Button("Download", GUILayout.Width(100)))
                    {
                        UnityWebRequest download = UnityWebRequest.Get(tasks.tasks[i].model_urls.glb);
                        fileName = tasks.tasks[i].name;
                        download.SendWebRequest().completed += DownloadCompleted;
                    }
                }
                // Left blank if not downloadable
                else if (tasks.tasks[i].started_at == 0)
                {
                    GUILayout.Label("", GUILayout.Width(100));
                }
                else
                {
                    GUILayout.Label("", GUILayout.Width(100));
                }
                GUILayout.Label(tasks.tasks[i].name, GUILayout.Width(100));
                GUILayout.Label(tasks.tasks[i].art_style, GUILayout.Width(150));
                GUILayout.Label("", GUILayout.Width(80));
                EditorGUI.ProgressBar(GUILayoutUtility.GetLastRect(), tasks.tasks[i].progress / 100.0f, tasks.tasks[i].progress.ToString());
                GUILayout.Label(tasks.tasks[i].status, GUILayout.Width(100));
                GUILayout.Label(startTime.AddMilliseconds(tasks.tasks[i].created_at).ToString(), GUILayout.Width(150));

                GUILayout.EndHorizontal();
            }
            GUILayout.EndVertical();
            GUILayout.EndScrollView();
        }
    }

}

/// <summary>
/// This model is used to submit the information of prompt.
/// For text to 3d.
/// </summary>
public class SubmitTextInfo : GUIModel
{

    [System.Serializable]
    public class TextTo3DRequest
    {
        public string object_prompt;
        public string style_prompt;
        public string negative_prompt;
        public string art_style;
        public bool enable_original_uv;
        public bool enable_pbr;
        public string name = "Meshy-model";
    }

    [System.Serializable]
    public class textureInfo
    {
        public string base_color;
    }

    private string[] atrStyleOptions = new string[]
    {
        "Realistic",
        "2.5D Cartoon",
        "Cartoon Line Art",
        "2.5D Hand-drawn",
        "Japanese Anime",
        "Realistic Hand-drawn",
        "Oriental Comic Lnk",
        "Voxel"
    };

    private string[] atrStyleValue = new string[]
    {
        "realistic",
        "fake-3d-cartoon",
        "cartoon-line-art",
        "fake-3d-hand-drawn",
        "japanese-anime",
        "realistic-hand-drawn",
        "oriental-comic-ink",
        "voxel"
    };

    private string objectPrompt;
    private string stylePrompt;
    private string negativePrompt;
    private int artStyle;
    private bool enableOriginalUv;
    private bool enablePbr;
    private string taskName = "Meshy-model";

    private static string OBJECT_PROMPT_UI_NAME = "Object Prompt";
    private static string STYLE_PROMPT_UI_NAME = "Style Prompt";
    static string ENALE_ORIGINAL_UV_UI_NAME = "Enable Orginal UV";
    private static string ENABLE_PBR_UI_NAME = "Enable PBR";
    private static string ART_STYLE_UI_NAME = "Art Style";
    private static string NEGATIVE_PROMPT_UI_NAME = "Negative Prompt";
    private static string TASK_NAME_UI_NAME = "Task Name";

    private TextTo3DRequest textTo3DRequest = null;

    void OnRequestCompleted(AsyncOperation operation)
    {
        UnityWebRequest request = ((UnityWebRequestAsyncOperation)operation).webRequest;
        if (request.result == UnityWebRequest.Result.Success)
        {
            if (request.downloadHandler != null)
            {
                string submittedTaskInfo = request.downloadHandler.text;
                Debug.Log(submittedTaskInfo);
            }
            else
            {
                Debug.Log("No response.");
            }
        }
        else
        {
            CheckErrorCode(request.responseCode,request.downloadHandler.text);
        }
    }

    public override void WebRequest()
    {
        textTo3DRequest = new TextTo3DRequest();
        textTo3DRequest.art_style = atrStyleValue[artStyle];
        textTo3DRequest.enable_pbr = enablePbr;
        textTo3DRequest.object_prompt = objectPrompt;
        textTo3DRequest.negative_prompt = negativePrompt;
        textTo3DRequest.style_prompt = stylePrompt;
        textTo3DRequest.enable_original_uv = enableOriginalUv;
        textTo3DRequest.name = taskName;
        string json = JsonUtility.ToJson(textTo3DRequest);
        byte[] bodyRow = System.Text.Encoding.UTF8.GetBytes(json);

        UnityWebRequest request = new UnityWebRequest("https://api.meshy.ai/v1/text-to-3d", "POST");
        request.SetRequestHeader("Authorization", EditorPrefs.GetString(API_KEY_FIELD));
        request.uploadHandler = new UploadHandlerRaw(bodyRow);
        request.downloadHandler = new DownloadHandlerBuffer();
        request.SendWebRequest().completed += OnRequestCompleted;
    }

    public override void Draw()
    {
        objectPrompt = EditorGUILayout.TextField(OBJECT_PROMPT_UI_NAME, objectPrompt);
        stylePrompt = EditorGUILayout.TextField(STYLE_PROMPT_UI_NAME, stylePrompt);
        negativePrompt = EditorGUILayout.TextField(NEGATIVE_PROMPT_UI_NAME, negativePrompt);
        artStyle = EditorGUILayout.Popup(ART_STYLE_UI_NAME, artStyle, atrStyleOptions);
        enablePbr = EditorGUILayout.Toggle(ENABLE_PBR_UI_NAME, enablePbr);
        enableOriginalUv = EditorGUILayout.Toggle(ENALE_ORIGINAL_UV_UI_NAME, enableOriginalUv);
        if (taskName == null || taskName == "")
        {
            taskName = "Meshy-model";
        }
        taskName = EditorGUILayout.TextField(TASK_NAME_UI_NAME, taskName);
        if (GUILayout.Button("Submit Task"))
        {
            WebRequest();
        }

    }
}
#endif
