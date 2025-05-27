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

public class APIKeyWindow : EditorWindow
{
    static string API_KEY_FIELD = "Meshy API Keys";
    private string APIKey;
    
    [MenuItem("Window/Meshy/API Key")]
    public static void ShowMeshyWindow()
    {
        APIKeyWindow wnd = GetWindow<APIKeyWindow>();
        wnd.titleContent = new GUIContent("Meshy API Key");
        if (EditorPrefs.HasKey(API_KEY_FIELD))
        {
            wnd.APIKey = EditorPrefs.GetString(API_KEY_FIELD);
        }
        else
        {
            wnd.APIKey = "";
        }
    }

    private void OnGUI()
    {
        GUIStyle textStyle = new GUIStyle(GUI.skin.textField);
        if (!EditorPrefs.HasKey(API_KEY_FIELD) || APIKey == null || APIKey == "")
        {
            textStyle.normal.textColor = Color.red;
            textStyle.hover.textColor = Color.red;
            textStyle.focused.textColor = Color.red;
        }
        else if (APIKey != EditorPrefs.GetString(API_KEY_FIELD))
        {
            textStyle.normal.textColor = Color.red;
            textStyle.hover.textColor = Color.red;
            textStyle.focused.textColor = Color.red;
        }
        else
        {
            Color lightGreen = new(0.105f, 0.62f, 0.25f);
            textStyle.normal.textColor = lightGreen;
            textStyle.hover.textColor = lightGreen;
            textStyle.focused.textColor = lightGreen;
        }
        
        APIKey = EditorGUILayout.TextField("API Key", APIKey, textStyle);
        if (GUILayout.Button("Save Key"))
        {
            EditorPrefs.SetString(API_KEY_FIELD, APIKey);
            Debug.Log("Save API key successfully!");
        }
    }
}
#endif