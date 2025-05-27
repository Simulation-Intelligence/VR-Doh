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


public class TextToTextureWindow : EditorWindow
{
    SubmitMeshInfo submitMeshInfo;
    GetTaskList getTaskList;
    float timer = 5;

    private void OnInspectorUpdate()
    {
        timer += 0.167f;
        if(timer >= 5.0f)
        {
            if (getTaskList.isRefreshing == true)
            {
                getTaskList.Refresh();
            }
            timer = 0;
        }
    }

    [MenuItem("Window/Meshy/Text To Texture")]
    public static void ShowMeshyWindow()
    {
        TextToTextureWindow wnd = GetWindow<TextToTextureWindow>();
        wnd.titleContent = new GUIContent("Text To Texture");
    }

    [MenuItem("GameObject/Meshy/Text To Texture", false, 100)]
    public static void ShowMeshyWindowWhenSelectGameObject()
    {
        TextToTextureWindow wnd = GetWindow<TextToTextureWindow>();
        wnd.titleContent = new GUIContent("Text To Texture");
        if (Selection.activeGameObject?.GetComponent<MeshFilter>() != null)
        {
            wnd.submitMeshInfo.selectedObject = Selection.activeGameObject;
        }
    }
    
    [MenuItem("GameObject/Meshy/Text To Texture", true)]
    private static bool ValidateShowMeshyWindowWhenSelectGameObject()
    {
        return Selection.gameObjects.Length == 1;
    }

    [MenuItem("Assets/Meshy/Text To Texture", false, 100)]
    public static void ShowMeshyWindowWhenSelectAsset()
    {
        TextToTextureWindow wnd = GetWindow<TextToTextureWindow>();
        wnd.titleContent = new GUIContent("Text To Texture");
        if (Selection.activeGameObject?.GetComponent<MeshFilter>() != null)
        {
            wnd.submitMeshInfo.selectedObject = Selection.activeGameObject;
        }
    }

    [MenuItem("Assets/Meshy/Text To Texture", true)]
    private static bool ValidateShowMeshyWindowWhenSelectAsset()
    {
        return Selection.gameObjects.Length == 1;
    }

    private void Awake()
    {
        getTaskList = new GetTaskList("https://api.meshy.ai/v1/text-to-texture");
        submitMeshInfo = new SubmitMeshInfo();
    }

    private void OnGUI()
    {
        submitMeshInfo.Draw();
        getTaskList.Draw();
    }
}
#endif