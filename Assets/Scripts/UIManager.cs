using System;
using System.IO;
using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.InputSystem;
using UnityEngine.EventSystems;
using UnityEngine.UI;
using TMPro;
using Oculus.Interaction;

class UIManager : MonoBehaviour
{
    public GameObject Mpm3DObject;
    public GameObject Mpm3DObject_2;
    // public GameObject Mpm3DObject_3;
    // public GameObject Mpm3DObject_4;
    // public GameObject Mpm3DObject_5;
    // public GameObject Mpm3DObject_6;

    [SerializeField]
    private SmoothHand leftSmoothHand;
    [SerializeField]
    private SmoothHand rightSmoothHand;
    [SerializeField]
    private GameObject MatTool_Hand_Left;
    [SerializeField]
    private GameObject MatTool_Hand_Right;
    public PinchGesture pinchGestureLeft;
    public PinchGesture pinchGestureRight;

    [SerializeField]
    private GameObject colorPickerObject;
    private ColorPicker colorPicker;
    public GameObject ShapeParameterObject_1;
    public GameObject ShapeParameterObject_2;
    private List<GameObject> createdObjectLists = new List<GameObject>();
    private List<GameObject> removedObjectLists = new List<GameObject>(); // Merged objects are removed
    private GameObject selectedObject;
    private GameObject prevSelectedObject;
    private string prefabName;
    public string export_folder_path;
    private string export_file_path;
    private bool EnableObjectGrab = true;

    // UI Components
    public GameObject UI_canvas;
    public Transform UI_anchor;
    private Vector3 canvas_anchor_offset;
    [SerializeField]
    private Camera sceneCamera;
    [SerializeField]
    private OVRHand[] oculus_hands;
    [SerializeField]
    private OVRSkeleton[] oculus_skeletons;
    public Button[] buttons;
    public GameObject mergePrompt;
    public GameObject valueAdjustGridSize;
    public GameObject valueAdjustSmoothness;
    private bool isMerging = false;
    public Toggle[] toggles;
    public TMP_Dropdown[] dropdowns;
    public InputField[] inputFields;
    public TMP_InputField[] tmpInputFields;
    public GameObject[] parameterObjects;
    public TouchScreenKeyboard overlayKeyboard;

    // Tools
    private Dictionary<string, MpmTool> mpmToolDict = new Dictionary<string, MpmTool>();
    private Dictionary<string, MatTool> matToolDict = new Dictionary<string, MatTool>();
    private string prevLeftHandTool;
    private string prevRightHandTool;

    void Awake()
    {
        canvas_anchor_offset = UI_canvas.transform.position - UI_anchor.position;

        // Load UI components
        foreach (Button button in buttons)
        {
            button.onClick.AddListener(() => OnButtonClick(button));
        }
        foreach (Toggle toggle in toggles)
        {
            toggle.onValueChanged.AddListener((bool isOn) => OnToggleValueChanged(toggle, isOn));
        }
        foreach (TMP_Dropdown dropdown in dropdowns)
        {
            dropdown.onValueChanged.AddListener((int value) => OnDropdownValueChanged(dropdown, value));
        }
        foreach (InputField inputField in inputFields)
        {
            inputField.onValueChanged.AddListener((string value) => OnInputFieldSelect(inputField));
        }
        foreach (TMP_InputField tmpInputField in tmpInputFields)
        {
            tmpInputField.onSelect.AddListener((string value) => OnTMPInputFieldSelect(tmpInputField));
        }
        foreach (GameObject parameter in parameterObjects)
        {
            TMP_Text parameter_text = parameter.transform.Find("Name").GetComponent<TMP_Text>();
            Slider parameter_slider = parameter.GetComponentInChildren<Slider>();

            // Adjust slider value by granularity
            float granularityDivisor = 50f;
            float minValue = parameter_slider.minValue;
            float maxValue = parameter_slider.maxValue;
            float range = maxValue - minValue;
            float granularity = range / granularityDivisor;

            // Change slider value granularity
            if (parameter_text != null && parameter_slider != null)
            {
                string initial_text = parameter_text.text;
                // float initial_value = Mathf.Round(parameter_slider.value / granularity) * granularity;
                float initial_value = parameter_slider.value;
                if (initial_text == "MaxDt" || initial_text == "FrameTime")
                {
                    parameter_text.text = initial_text + ": " + initial_value.ToString();
                    parameter_slider.value = initial_value;
                    granularity = initial_value;
                    parameter_slider.onValueChanged.AddListener((float value) =>
                    {
                        float adjusted_value = Mathf.Round(value / granularity) * granularity;
                        parameter_text.text = initial_text + ": " + adjusted_value.ToString();
                        parameter_slider.value = adjusted_value;
                    });
                }
                else
                {
                    parameter_text.text = initial_text + ": " + initial_value.ToString("F3");
                    parameter_slider.value = initial_value;
                    parameter_slider.onValueChanged.AddListener((float value) =>
                    {
                        float adjusted_value = Mathf.Round(value / granularity) * granularity;
                        parameter_text.text = initial_text + ": " + adjusted_value.ToString("F3");
                        parameter_slider.value = adjusted_value;
                    });
                }
            }
        }

        InstantiateTools(); // Instantiate tools
        AddMpm3DObject();
    }

    void AddMpm3DObject()
    {
        if (Mpm3DObject != null)
        {
            createdObjectLists.Add(Mpm3DObject);
            // Use the just created object as the selected object for further interactions
            selectedObject = Mpm3DObject;
            // Select tools for modeling
            SelectTools(selectedObject, "MatTool_Hand_Left", "MatTool_Hand_Right");
            ShowSelectedObjectCanvas();
        }
        if (Mpm3DObject_2 != null)
        {
            createdObjectLists.Add(Mpm3DObject_2);
            SelectTools(Mpm3DObject_2, "MatTool_Hand_Left", "MatTool_Hand_Right");
        }
        // if (Mpm3DObject_3 != null)
        // {
        //     createdObjectLists.Add(Mpm3DObject_3);
        //     SelectTools(Mpm3DObject_2, "MatTool_Hand_Left", "MatTool_Hand_Right");
        // }
        // if (Mpm3DObject_4 != null)
        // {
        //     createdObjectLists.Add(Mpm3DObject_4);
        //     SelectTools(Mpm3DObject_2, "MatTool_Hand_Left", "MatTool_Hand_Right");
        // }
        // if (Mpm3DObject_5 != null)
        // {
        //     createdObjectLists.Add(Mpm3DObject_5);
        //     SelectTools(Mpm3DObject_2, "MatTool_Hand_Left", "MatTool_Hand_Right");
        // }
        // if (Mpm3DObject_6 != null)
        // {
        //     createdObjectLists.Add(Mpm3DObject_6);
        //     SelectTools(Mpm3DObject_2, "MatTool_Hand_Left", "MatTool_Hand_Right");
        // }
    }

    void InstantiateTools()
    {
        // Instantiate all mat tools at the beginning, either hands gameobject or prefabs
        matToolDict.Add("MatTool_Hand_Left", MatTool_Hand_Left.GetComponent<MatTool>());
        matToolDict.Add("MatTool_Hand_Right", MatTool_Hand_Right.GetComponent<MatTool>());
        matToolDict.Add("MatTool_Pad_Left", Instantiate(Resources.Load<GameObject>("Prefabs/Tools/MatToolPrefab_Pad_Left")).GetComponent<MatTool>());
        matToolDict.Add("MatTool_Pad_Right", Instantiate(Resources.Load<GameObject>("Prefabs/Tools/MatToolPrefab_Pad_Right")).GetComponent<MatTool>());
        matToolDict.Add("MatTool_Rod_Left", Instantiate(Resources.Load<GameObject>("Prefabs/Tools/MatToolPrefab_Rod_Left")).GetComponent<MatTool>());
        matToolDict.Add("MatTool_Rod_Right", Instantiate(Resources.Load<GameObject>("Prefabs/Tools/MatToolPrefab_Rod_Right")).GetComponent<MatTool>());
        matToolDict.Add("MatTool_Cone_Left", Instantiate(Resources.Load<GameObject>("Prefabs/Tools/MatToolPrefab_Cone_Left")).GetComponent<MatTool>());
        matToolDict.Add("MatTool_Cone_Right", Instantiate(Resources.Load<GameObject>("Prefabs/Tools/MatToolPrefab_Cone_Right")).GetComponent<MatTool>());
        matToolDict.Add("MatTool_Slab_Left", Instantiate(Resources.Load<GameObject>("Prefabs/Tools/MatToolPrefab_Slab_Left")).GetComponent<MatTool>());
        matToolDict.Add("MatTool_Slab_Right", Instantiate(Resources.Load<GameObject>("Prefabs/Tools/MatToolPrefab_Slab_Right")).GetComponent<MatTool>());
        matToolDict.Add("MatTool_Scissor_Left", Instantiate(Resources.Load<GameObject>("Prefabs/Tools/MatToolPrefab_Scissor_Left")).GetComponent<MatTool>());
        matToolDict.Add("MatTool_Scissor_Right", Instantiate(Resources.Load<GameObject>("Prefabs/Tools/MatToolPrefab_Scissor_Right")).GetComponent<MatTool>());

        foreach (var matTool in matToolDict.Values)
        {
            matTool.transform.SetParent(transform);
            matTool.gameObject.SetActive(false);

            // Specify SmoothHand object to MatTools
            if (matTool.handType == MatTool.HandType.LeftHand)
            {
                matTool.smoothHand = leftSmoothHand;
            }
            else if (matTool.handType == MatTool.HandType.RightHand)
            {
                matTool.smoothHand = rightSmoothHand;
            }
        }

        // Capsule based mpm tools
        // mpmToolDict.Add("Tool_LeftHand", Instantiate(Resources.Load<GameObject>("Prefabs/Tools/ToolPrefab_LeftHand")).GetComponent<MpmTool>());
        // mpmToolDict.Add("Tool_RightHand", Instantiate(Resources.Load<GameObject>("Prefabs/Tools/ToolPrefab_RightHand")).GetComponent<MpmTool>());
        // mpmToolDict.Add("Tool_Sphere_LeftHand", Instantiate(Resources.Load<GameObject>("Prefabs/Tools/ToolPrefab_Sphere_LeftHand")).GetComponent<MpmTool>());
        // mpmToolDict.Add("Tool_Sphere_RightHand", Instantiate(Resources.Load<GameObject>("Prefabs/Tools/ToolPrefab_Sphere_RightHand")).GetComponent<MpmTool>());
        // foreach (var mpmTool in mpmToolDict.Values)
        // {
        //     mpmTool.transform.SetParent(transform);
        //     mpmTool.gameObject.SetActive(false);
        // }
    }

    void SelectTools(GameObject slectedMpm3DObject, string leftHandTool, string rightHandTool)
    {
        // Use hands as the default tool
        Mpm3DMarching mpm3DSimulation = slectedMpm3DObject.GetComponent<Mpm3DMarching>();
        mpm3DSimulation.matTools.Clear();

        // Add left hand tool and set active
        if (!string.IsNullOrEmpty(prevLeftHandTool) && prevLeftHandTool != leftHandTool)
        {
            matToolDict[prevLeftHandTool].gameObject.SetActive(false);
        }
        matToolDict[leftHandTool].gameObject.SetActive(true);
        mpm3DSimulation.matTools.Add(matToolDict[leftHandTool]);

        // Add right hand tool and set active
        if (!string.IsNullOrEmpty(prevRightHandTool) && prevRightHandTool != rightHandTool)
        {
            matToolDict[prevRightHandTool].gameObject.SetActive(false);
        }
        matToolDict[rightHandTool].gameObject.SetActive(true);
        mpm3DSimulation.matTools.Add(matToolDict[rightHandTool]);

        // Initialize tools for simulation
        mpm3DSimulation.Init_MatTools();

        prevLeftHandTool = leftHandTool;
        prevRightHandTool = rightHandTool;
    }

    void OnColorChanged(Color newColor)
    {
        // Used by color picker to adjust color
        if (selectedObject != null)
        {
            Mpm3DMarching mpm3DSimulation = selectedObject.GetComponent<Mpm3DMarching>();
            if (mpm3DSimulation != null)
                mpm3DSimulation.AdjustTextureColor(newColor);
        }
    }

    void Update()
    {
        // Find the last grabbed object as the selected object for further manipulations
        if (createdObjectLists.Count > 0)
        {
            foreach (var createdObject in createdObjectLists)
            {
                var _grabbable = createdObject.GetComponent<Grabbable>();
                if (_grabbable.SelectingPointsCount > 0)
                {
                    // Merge the object with another object
                    if (isMerging)
                    {
                        GameObject objectToMerge = createdObject;
                        if (selectedObject != null && objectToMerge != null && selectedObject != objectToMerge)
                        {
                            Mpm3DMarching mpm3DSimulation = selectedObject.GetComponent<Mpm3DMarching>();
                            if (mpm3DSimulation != null)
                            {
                                mpm3DSimulation.MergeAndUpdate(objectToMerge.GetComponent<Mpm3DMarching>());
                                Debug.Log("Merge object " + selectedObject.name + " with object " + objectToMerge.name);
                                isMerging = false;
                                mergePrompt.SetActive(false);
                                //removedObjectLists.Add(objectToMerge); // Need to distroy the merged object
                                objectToMerge = null;
                                foreach (Button button in buttons)
                                {
                                    if (button.name == "Button_MergeObject")
                                    {
                                        button.GetComponentInChildren<TMP_Text>().text = "Merge";
                                    }
                                }
                            }
                        }
                    }
                    else
                    {
                        // Update object-wise data in UI
                        selectedObject = createdObject;
                        if (selectedObject != prevSelectedObject)
                        {
                            ShowSelectedObjectCanvas();
                            prevSelectedObject = selectedObject;
                        }
                    }
                    break;
                }
            }

            if (selectedObject != null)
            {
                // Visualize pinch gesture spheres when enabled
                pinchGestureLeft.RenderPinchSphere = selectedObject.GetComponent<Mpm3DMarching>().UsePinchGestureLeft;
                pinchGestureRight.RenderPinchSphere = selectedObject.GetComponent<Mpm3DMarching>().UsePinchGestureRight;

                pinchGestureLeft.RenderRotationSphere = selectedObject.GetComponent<Mpm3DMarching>().UsePinchGestureLeft;
                pinchGestureRight.RenderRotationSphere = selectedObject.GetComponent<Mpm3DMarching>().UsePinchGestureRight;

                // Disable object grab when pinch gesture is enabled, avoiding unexpected rotation
                if (EnableObjectGrab)
                {
                    if ((pinchGestureLeft.RenderPinchSphere && pinchGestureLeft.isPinching) || (pinchGestureRight.RenderPinchSphere && pinchGestureRight.isPinching)
                     || (pinchGestureLeft.RenderRotationSphere && pinchGestureLeft.isRotating) || (pinchGestureRight.RenderRotationSphere && pinchGestureRight.isRotating))
                    {
                        var _grabbable = selectedObject.GetComponent<Grabbable>();
                        _grabbable.MaxGrabPoints = 0;
                    }
                    else
                    {
                        var _grabbable = selectedObject.GetComponent<Grabbable>();
                        _grabbable.MaxGrabPoints = -1;
                    }
                }
            }
        }

        // Distroy the removed objects
        if (removedObjectLists.Count > 0)
        {
            foreach (var removedObject in removedObjectLists)
            {
                createdObjectLists.Remove(removedObject);
                removedObject.GetComponent<Mpm3DMarching>().Dispose();
                Destroy(removedObject);
            }
            removedObjectLists.Clear();
        }
    }

    void CreateNewMpm3DObject()
    {
        if (Mpm3DObject != null)
        {
            // Position and name
            Vector3 position = sceneCamera.transform.position + sceneCamera.transform.forward * 0.2f;
            position.x -= 0.1f;
            position.y -= 0.2f;
            Quaternion rotation = Quaternion.LookRotation(sceneCamera.transform.forward);

            GameObject newMpm3DObject = Instantiate(Mpm3DObject, position, rotation);
            createdObjectLists.Add(newMpm3DObject);

            // Use the just created object as the selected object for further interactions
            selectedObject = newMpm3DObject;
            newMpm3DObject.name = "Mpm3DObject_" + createdObjectLists.Count;

            // Initialize the object
            newMpm3DObject.GetComponent<Mpm3DMarching>().Initiate();
            // Select tools for modeling
            SelectTools(selectedObject, "MatTool_Hand_Left", "MatTool_Hand_Left");
            // Apply materials specified from UI
            // ApplyMaterial(newMpm3DObject);
        }
    }

    void ApplyMaterial(GameObject slectedMpm3DObject)
    {
        // Store the object parameters when creating the object
        Mpm3DMarching mpm3DSimulation = slectedMpm3DObject.GetComponent<Mpm3DMarching>();

        // Update the grid size in UI
        string initial_text = valueAdjustGridSize.GetComponent<TMP_Text>().text;
        int grid_size = mpm3DSimulation.GetGridSize();
        string new_text = initial_text.Substring(0, initial_text.IndexOf(":") + 2) + (grid_size).ToString();
        valueAdjustGridSize.GetComponent<TMP_Text>().text = new_text;

        // Initialize or update the object parameters
        foreach (TMP_Dropdown dropdown in dropdowns)
        {
            if (dropdown.name == "Dropdown_MaterialType")
            {
                mpm3DSimulation.materialType = (Mpm3DMarching.MaterialType)Enum.Parse(typeof(Mpm3DMarching.MaterialType), dropdown.value.ToString());
            }
            if (dropdown.name == "Dropdown_PlasticityType")
            {
                mpm3DSimulation.plasticityType = (Mpm3DMarching.PlasticityType)Enum.Parse(typeof(Mpm3DMarching.PlasticityType), dropdown.value.ToString());
            }
            if (dropdown.name == "Dropdown_StressType")
            {
                mpm3DSimulation.stressType = (Mpm3DMarching.StressType)Enum.Parse(typeof(Mpm3DMarching.StressType), dropdown.value.ToString());
            }
        }

        foreach (GameObject parameter in parameterObjects)
        {
            if (parameter.name == "Parameter_E")
            {
                mpm3DSimulation._E = parameter.GetComponentInChildren<Slider>().value;
            }
            if (parameter.name == "Parameter_SigY")
            {
                mpm3DSimulation._SigY = parameter.GetComponentInChildren<Slider>().value;
            }
            if (parameter.name == "Parameter_Nu")
            {
                mpm3DSimulation._nu = parameter.GetComponentInChildren<Slider>().value;
            }
            if (parameter.name == "Parameter_CollideFactor")
            {
                mpm3DSimulation.colide_factor = parameter.GetComponentInChildren<Slider>().value;
            }
            if (parameter.name == "Parameter_FrictionK")
            {
                mpm3DSimulation.friction_k = parameter.GetComponentInChildren<Slider>().value;
            }
            if (parameter.name == "Parameter_P_Rho")
            {
                mpm3DSimulation.p_rho = parameter.GetComponentInChildren<Slider>().value;
            }
            if (parameter.name == "Parameter_FrictionAngle")
            {
                mpm3DSimulation.friction_angle = parameter.GetComponentInChildren<Slider>().value;
            }
            if (parameter.name == "Parameter_Damping")
            {
                mpm3DSimulation.damping = parameter.GetComponentInChildren<Slider>().value;
            }
            if (parameter.name == "Parameter_nGrid")
            {
                mpm3DSimulation.SetSimulateGridSize((int)parameter.GetComponentInChildren<Slider>().value);
            }
            if (parameter.name == "Parameter_ParticlePerGrid")
            {
                mpm3DSimulation.particle_per_grid = parameter.GetComponentInChildren<Slider>().value;
            }
            if (parameter.name == "Parameter_MaxDt")
            {
                mpm3DSimulation.max_dt = parameter.GetComponentInChildren<Slider>().value;
            }
            if (parameter.name == "Parameter_FrameTime")
            {
                mpm3DSimulation.frame_time = parameter.GetComponentInChildren<Slider>().value;
            }
            if (parameter.name == "Parameter_AllowedCFL")
            {
                mpm3DSimulation.allowed_cfl = parameter.GetComponentInChildren<Slider>().value;
            }
            if (parameter.name == "Parameter_HandSimulationRadius")
            {
                mpm3DSimulation.SetHandsimulationRadius(parameter.GetComponentInChildren<Slider>().value);
            }
        }

        Debug.Log("materialType: " + mpm3DSimulation.materialType);
        Debug.Log("plasticityType: " + mpm3DSimulation.plasticityType);
        Debug.Log("stressType: " + mpm3DSimulation.stressType);
        Debug.Log("E: " + mpm3DSimulation._E);
        Debug.Log("SigY: " + mpm3DSimulation._SigY);
        Debug.Log("nu: " + mpm3DSimulation._nu);
        Debug.Log("colide_factor: " + mpm3DSimulation.colide_factor);
        Debug.Log("friction_k: " + mpm3DSimulation.friction_k);
        Debug.Log("p_rho: " + mpm3DSimulation.p_rho);
        Debug.Log("friction_angle: " + mpm3DSimulation.friction_angle);
        Debug.Log("damping: " + mpm3DSimulation.damping);
        Debug.Log("n_grid: " + mpm3DSimulation.GetGridSize());

        // Update materials to simulation
        mpm3DSimulation.Init_materials();
        mpm3DSimulation.Copy_materials();
    }

    void OnButtonClick(Button button)
    {
        Debug.Log(button.name + " was clicked!");
        if (button.GetComponentInChildren<TMP_Text>().text == "Apply")
        {
            button.GetComponentInChildren<TMP_Text>().text = "Applyed!";
        }
        else if (button.GetComponentInChildren<TMP_Text>().text == "Applyed!")
        {
            button.GetComponentInChildren<TMP_Text>().text = "Apply";
        }

        // Create a new object in the scene
        if (button.name == "Button_CreateObject")
        {
            // May need to have a file brower
            // CreateNewMpm3DObject();
        }
        // Create object based on shape parameters
        if (button.name == "Button_Confirm")
        {
            ShapeParameterObject_1.SetActive(false);
            ShapeParameterObject_2.SetActive(false);
            CreateMpm3DObjectFromPrefab();
        }
        // Adjust materials during the modeling process
        if (button.name == "Button_ApplyMaterials")
        {
            if (selectedObject != null)
            {
                // ApplyMaterial(selectedObject);
            }
        }
        // Merge the object with another object
        if (button.name == "Button_MergeObject")
        {
            if (selectedObject != null)
            {
                if (button.GetComponentInChildren<TMP_Text>().text == "Merge")
                {
                    button.GetComponentInChildren<TMP_Text>().text = "Cancel Merging";
                    mergePrompt.SetActive(true);
                    mergePrompt.GetComponent<TMP_Text>().text = "Please select an object to merge";
                    isMerging = true;
                }
                else
                {
                    button.GetComponentInChildren<TMP_Text>().text = "Merge";
                    mergePrompt.SetActive(false);
                    isMerging = false;
                }
            }
        }
        // Copy the workong-on-progress object during the modeling process
        if (button.name == "Button_CopyObject")
        {
            if (selectedObject != null)
            {
                Vector3 newPosition = selectedObject.transform.position + new Vector3(0.2f, 0.0f, 0.0f);
                Quaternion newRotation = selectedObject.transform.rotation;
                GameObject copiedObject = Instantiate(selectedObject, newPosition, newRotation);
                copiedObject.transform.SetParent(selectedObject.transform.parent, false);
                createdObjectLists.Add(copiedObject);
                copiedObject.name = "Mpm3DObject_" + createdObjectLists.Count;
                // Copy particles and materials
                Mpm3DMarching mpm3DSimulation = copiedObject.GetComponent<Mpm3DMarching>();
                selectedObject.GetComponent<Mpm3DMarching>().CopyObjectTo(mpm3DSimulation);
                selectedObject = copiedObject;
                // Select tools for modeling
                SelectTools(selectedObject, prevLeftHandTool, prevRightHandTool);
            }
        }
        // Reset the object with original parameters
        if (button.name == "Button_ResetObject")
        {
            if (selectedObject != null)
            {
                // Store the original color, position, scale, rotation, and prefab name
                Mpm3DMarching mpm3DSimulation = selectedObject.GetComponent<Mpm3DMarching>();
                mpm3DSimulation.Reset();

                // Reset from prefab
                // Vector3 originalPosition = selectedObject.transform.position;
                // Vector3 originalScale = selectedObject.transform.localScale;
                // Quaternion originalRotation = selectedObject.transform.rotation;
                // string prefabName = selectedObject.GetComponent<Mpm3DMarching>().initShape.ToString();
                // Color originalColor = selectedObject.transform.Find("MarchingCubeVisualizer").gameObject.GetComponent<MeshRenderer>().material.color;

                // // Destroy the object
                // createdObjectLists.Remove(selectedObject);
                // Destroy(selectedObject);

                // // Create a new object
                // GameObject Mpm3DObject = Resources.Load<GameObject>("Prefabs/Mpm3DMarching" + prefabName);
                // if (Mpm3DObject != null)
                // {
                //     GameObject newMpm3DObject = Instantiate(Mpm3DObject, originalPosition, originalRotation);
                //     newMpm3DObject.transform.localScale = originalScale;
                //     createdObjectLists.Add(newMpm3DObject);
                //     selectedObject = newMpm3DObject;
                //     newMpm3DObject.name = "Mpm3DObject_" + createdObjectLists.Count;
                //     Mpm3DMarching mpm3DSimulation = selectedObject.GetComponent<Mpm3DMarching>();
                //     mpm3DSimulation.AdjustTextureColor(originalColor);
                //     ApplyMaterial(newMpm3DObject);
                // }
            }
        }
        if (button.name == "Button_Pinch_Selection_Radius")
        {
            if (selectedObject != null)
            {
                Mpm3DMarching mpm3DSimulation = selectedObject.GetComponent<Mpm3DMarching>();
                foreach (GameObject parameter in parameterObjects)
                {
                    if (parameter.name == "Parameter_Pinch_Selection_Radius")
                    {
                        pinchGestureLeft.pinchRadius = parameter.GetComponentInChildren<Slider>().value;
                        pinchGestureRight.pinchRadius = parameter.GetComponentInChildren<Slider>().value;
                    }
                }
            }
        }
        if (button.name == "Button_Pinch_Force")
        {
            if (selectedObject != null)
            {
                Mpm3DMarching mpm3DSimulation = selectedObject.GetComponent<Mpm3DMarching>();
                foreach (GameObject parameter in parameterObjects)
                {
                    if (parameter.name == "Parameter_Pinch_Force")
                    {
                        mpm3DSimulation.SetPinchForceRatio(parameter.GetComponentInChildren<Slider>().value);
                    }
                }
            }
        }
        // Delete the object from the scene
        if (button.name == "Button_DeleteObject")
        {
            if (selectedObject != null)
            {
                createdObjectLists.Remove(selectedObject);
                Destroy(selectedObject);
                selectedObject = null;
            }
        }
        // Export data to a file
        if (button.name == "Button_ExportObject")
        {
            if (export_folder_path != null)
            {
                if (selectedObject != null)
                {
                    Mpm3DMarching mpm3DSimulation = selectedObject.GetComponent<Mpm3DMarching>();
                    export_file_path = export_folder_path + "/" + selectedObject.name + ".txt";
                    mpm3DSimulation.ExportData(export_file_path);

                    // Show file name in UI
                    foreach (Button _button in buttons)
                    {
                        if (_button.name == "Button_ExportObject")
                        {
                            string initialText = _button.GetComponentInChildren<TMP_Text>().text;
                            _button.GetComponentInChildren<TMP_Text>().text = initialText + "(" + selectedObject.name + ".txt)";
                        }
                    }
                }
            }
            else
            {
                // Inform no export folder in UI
                foreach (Button _button in buttons)
                {
                    if (_button.name == "Button_ExportObject")
                    {
                        string initialText = _button.GetComponentInChildren<TMP_Text>().text;
                        _button.GetComponentInChildren<TMP_Text>().text = initialText + "(No Export Folder)";
                    }
                }
            }
        }
        // Adjust the smoothness of the marching cubes
        if (button.name == "Button_MoreSmooth")
        {
            if (selectedObject != null)
            {
                Mpm3DMarching mpm3DSimulation = selectedObject.GetComponent<Mpm3DMarching>();
                mpm3DSimulation.IncreaseSmoothingIterations();
                string initial_text_smooth = valueAdjustSmoothness.GetComponent<TMP_Text>().text;
                int smooth_value = mpm3DSimulation.GetSmoothingIterations(); // smooth iterations
                string new_text_smooth = initial_text_smooth.Substring(0, initial_text_smooth.IndexOf(":") + 2) + (smooth_value).ToString();
                valueAdjustSmoothness.GetComponent<TMP_Text>().text = new_text_smooth;
            }
        }
        if (button.name == "Button_LessSmooth")
        {
            if (selectedObject != null)
            {
                Mpm3DMarching mpm3DSimulation = selectedObject.GetComponent<Mpm3DMarching>();
                mpm3DSimulation.DecreaseSmoothingIterations();
                string initial_text_smooth = valueAdjustSmoothness.GetComponent<TMP_Text>().text;
                int smooth_value = mpm3DSimulation.GetSmoothingIterations(); // smooth iterations
                string new_text_smooth = initial_text_smooth.Substring(0, initial_text_smooth.IndexOf(":") + 2) + (smooth_value).ToString();
                valueAdjustSmoothness.GetComponent<TMP_Text>().text = new_text_smooth;
            }
        }
        // Increase or decrease grid size for marching cubes
        if (button.name == "Button_IncreaseGridSize")
        {
            if (selectedObject != null)
            {
                int increaseValue = 8;
                Mpm3DMarching mpm3DSimulation = selectedObject.GetComponent<Mpm3DMarching>();
                mpm3DSimulation.IncreaseGridSize(increaseValue);

                // Update the grid size in UI
                string initial_text = valueAdjustGridSize.GetComponent<TMP_Text>().text;
                string new_text = initial_text.Substring(0, initial_text.IndexOf(":") + 2) + mpm3DSimulation.GetGridSize().ToString();
                valueAdjustGridSize.GetComponent<TMP_Text>().text = new_text;
            }
        }
        if (button.name == "Button_DecreaseGridSize")
        {
            if (selectedObject != null)
            {
                int decreaseValue = 8;
                Mpm3DMarching mpm3DSimulation = selectedObject.GetComponent<Mpm3DMarching>();
                mpm3DSimulation.DecreaseGridSize(decreaseValue);

                // Update the grid size in UI
                string initial_text = valueAdjustGridSize.GetComponent<TMP_Text>().text;
                string new_text = initial_text.Substring(0, initial_text.IndexOf(":") + 2) + mpm3DSimulation.GetGridSize().ToString();
                valueAdjustGridSize.GetComponent<TMP_Text>().text = new_text;
            }
        }
        if (button.name == "Button_HandSimulationRadius")
        {
            if (selectedObject != null)
            {
                Mpm3DMarching mpm3DSimulation = selectedObject.GetComponent<Mpm3DMarching>();
                foreach (GameObject parameter in parameterObjects)
                {
                    if (parameter.name == "Parameter_HandSimulationRadius")
                    {
                        mpm3DSimulation.SetHandsimulationRadius(parameter.GetComponentInChildren<Slider>().value);
                    }
                }
            }
        }
        if (button.name == "Button_E")
        {
            if (selectedObject != null)
            {
                Mpm3DMarching mpm3DSimulation = selectedObject.GetComponent<Mpm3DMarching>();
                foreach (GameObject parameter in parameterObjects)
                {
                    if (parameter.name == "Parameter_E")
                    {
                        mpm3DSimulation._E = parameter.GetComponentInChildren<Slider>().value;
                        mpm3DSimulation.Init_materials();
                        mpm3DSimulation.Copy_materials();
                    }
                }
            }
        }
        if (button.name == "Button_SigY")
        {
            if (selectedObject != null)
            {
                Mpm3DMarching mpm3DSimulation = selectedObject.GetComponent<Mpm3DMarching>();
                foreach (GameObject parameter in parameterObjects)
                {
                    if (parameter.name == "Parameter_SigY")
                    {
                        mpm3DSimulation._SigY = parameter.GetComponentInChildren<Slider>().value;
                        mpm3DSimulation.Init_materials();
                        mpm3DSimulation.Copy_materials();
                    }
                }
            }
        }
        if (button.name == "Button_Damping")
        {
            if (selectedObject != null)
            {
                Mpm3DMarching mpm3DSimulation = selectedObject.GetComponent<Mpm3DMarching>();
                foreach (GameObject parameter in parameterObjects)
                {
                    if (parameter.name == "Parameter_Damping")
                    {
                        mpm3DSimulation.damping = parameter.GetComponentInChildren<Slider>().value;
                        mpm3DSimulation.Init_materials();
                        mpm3DSimulation.Copy_materials();
                    }
                }
            }
        }
        if (button.name == "Button_Nu")
        {
            if (selectedObject != null)
            {
                Mpm3DMarching mpm3DSimulation = selectedObject.GetComponent<Mpm3DMarching>();
                foreach (GameObject parameter in parameterObjects)
                {
                    if (parameter.name == "Parameter_Nu")
                    {
                        mpm3DSimulation._nu = parameter.GetComponentInChildren<Slider>().value;
                        mpm3DSimulation.Init_materials();
                        mpm3DSimulation.Copy_materials();
                    }
                }
            }
        }
        if (button.name == "Button_CollideFactor")
        {
            if (selectedObject != null)
            {
                Mpm3DMarching mpm3DSimulation = selectedObject.GetComponent<Mpm3DMarching>();
                foreach (GameObject parameter in parameterObjects)
                {
                    if (parameter.name == "Parameter_CollideFactor")
                    {
                        mpm3DSimulation.colide_factor = parameter.GetComponentInChildren<Slider>().value;
                        mpm3DSimulation.Init_materials();
                        mpm3DSimulation.Copy_materials();
                    }
                }
            }
        }
        if (button.name == "Button_FrictionK")
        {
            if (selectedObject != null)
            {
                Mpm3DMarching mpm3DSimulation = selectedObject.GetComponent<Mpm3DMarching>();
                foreach (GameObject parameter in parameterObjects)
                {
                    if (parameter.name == "Parameter_FrictionK")
                    {
                        mpm3DSimulation.friction_k = parameter.GetComponentInChildren<Slider>().value;
                        mpm3DSimulation.Init_materials();
                        mpm3DSimulation.Copy_materials();
                    }
                }
            }
        }
        if (button.name == "Button_P_Rho")
        {
            if (selectedObject != null)
            {
                Mpm3DMarching mpm3DSimulation = selectedObject.GetComponent<Mpm3DMarching>();
                foreach (GameObject parameter in parameterObjects)
                {
                    if (parameter.name == "Parameter_P_Rho")
                    {
                        mpm3DSimulation.p_rho = parameter.GetComponentInChildren<Slider>().value;
                        mpm3DSimulation.Init_materials();
                        mpm3DSimulation.Copy_materials();
                    }
                }
            }

        }
        if (button.name == "Button_FrictionAngle")
        {
            if (selectedObject != null)
            {
                Mpm3DMarching mpm3DSimulation = selectedObject.GetComponent<Mpm3DMarching>();
                foreach (GameObject parameter in parameterObjects)
                {
                    if (parameter.name == "Parameter_FrictionAngle")
                    {
                        mpm3DSimulation.friction_angle = parameter.GetComponentInChildren<Slider>().value;
                        mpm3DSimulation.Init_materials();
                        mpm3DSimulation.Copy_materials();
                    }
                }
            }
        }
        if (button.name == "Button_nGrid")
        {
            if (selectedObject != null)
            {
                Mpm3DMarching mpm3DSimulation = selectedObject.GetComponent<Mpm3DMarching>();
                foreach (GameObject parameter in parameterObjects)
                {
                    if (parameter.name == "Parameter_nGrid")
                    {
                        mpm3DSimulation.SetSimulateGridSize((int)parameter.GetComponentInChildren<Slider>().value);
                    }
                }
            }
        }
        if (button.name == "Button_MaxDt")
        {
            if (selectedObject != null)
            {
                Mpm3DMarching mpm3DSimulation = selectedObject.GetComponent<Mpm3DMarching>();
                foreach (GameObject parameter in parameterObjects)
                {
                    if (parameter.name == "Parameter_MaxDt")
                    {
                        mpm3DSimulation.max_dt = parameter.GetComponentInChildren<Slider>().value;
                        mpm3DSimulation.Init_materials();
                        mpm3DSimulation.Copy_materials();
                    }
                }
            }
        }
    }

    void OnToggleValueChanged(Toggle toggle, bool isOn)
    {
        Debug.Log("Toggle " + toggle.name + " is " + (isOn ? "On" : "Off"));

        // Enable/Disable pinch gesture for mid-air interactions
        if (toggle.name == "Toggle_EnablePinchGesture_Left")
        {
            if (selectedObject != null)
            {
                Mpm3DMarching mpm3DSimulation = selectedObject.GetComponent<Mpm3DMarching>();
                mpm3DSimulation.UsePinchGestureLeft = toggle.isOn;
            }
        }
        if (toggle.name == "Toggle_EnablePinchGesture_Right")
        {
            if (selectedObject != null)
            {
                Mpm3DMarching mpm3DSimulation = selectedObject.GetComponent<Mpm3DMarching>();
                mpm3DSimulation.UsePinchGestureRight = toggle.isOn;
            }
        }
        // Enable/Disable color picker object
        if (toggle.name == "Toggle_ColorPicker")
        {
            if (colorPickerObject != null)
            {
                colorPickerObject.SetActive(isOn);
                if (isOn)
                {
                    ColorPicker colorPicker = colorPickerObject.GetComponentInChildren<ColorPicker>();
                    colorPicker.onColorChanged += OnColorChanged;
                }
                else
                {
                    ColorPicker colorPicker = colorPickerObject.GetComponentInChildren<ColorPicker>();
                    colorPicker.onColorChanged -= OnColorChanged;
                }
            }
        }
        // Enable/Disable object grab
        if (toggle.name == "Toggle_EnableGrab")
        {
            if (selectedObject != null)
            {
                Text toggle_label = toggle.GetComponentInChildren<Text>();
                toggle_label.text = isOn ? "Object is grabbable" : "Object is not grabbable";
                var _grabbable = selectedObject.GetComponent<Grabbable>();
                _grabbable.MaxGrabPoints = isOn ? -1 : 0;
                EnableObjectGrab = isOn;
            }
        }
        // Enable/Disable object interaction for simulation to avoid unintended hand contacts
        if (toggle.name == "Toggle_EnableInteraction")
        {
            if (selectedObject != null)
            {
                Text toggle_label = toggle.GetComponentInChildren<Text>();
                toggle_label.text = isOn ? "Object is Interactable" : "Object is not Interactable";
                selectedObject.GetComponent<Mpm3DMarching>().RunSimulation = isOn;
            }
        }
        if (toggle.name == "Toggle_VisualBox")
        {
            if (selectedObject != null)
            {
                if (selectedObject.transform.Find("Visuals") != null)
                {
                    Transform VisualObject = selectedObject.transform.Find("Visuals");
                    MeshRenderer meshRenderer = VisualObject.Find("Mesh").GetComponent<MeshRenderer>();
                    meshRenderer.enabled = isOn;
                }
            }
        }
        // Enable/Disable fix object in place
        if (toggle.name == "Toggle_FixObject")
        {
            if (selectedObject != null)
            {
                Mpm3DMarching mpm3DSimulation = selectedObject.GetComponent<Mpm3DMarching>();
                mpm3DSimulation.SetFixed(toggle.isOn);
                Debug.Log("Object " + selectedObject.name + " is " + (toggle.isOn ? "Fixed" : "Unfixed"));
            }
        }
        // Use sticky boundary cnodition
        if (toggle.name == "Toggle_StickyGround")
        {
            if (selectedObject != null)
            {
                Mpm3DMarching mpm3DSimulation = selectedObject.GetComponent<Mpm3DMarching>();
                mpm3DSimulation.SetStickyBoundary(toggle.isOn);
            }
        }
        // Enable/Disable the gravity
        if (toggle.name == "Toggle_Gravity")
        {
            if (selectedObject != null)
            {
                Mpm3DMarching mpm3DSimulation = selectedObject.GetComponent<Mpm3DMarching>();
                mpm3DSimulation.SetGravity(isOn ? -50f : 0.0f);
            }
        }
        // Use correct CFL condition
        if (toggle.name == "Toggle_UseCorrectCFL")
        {
            if (selectedObject != null)
            {
                Mpm3DMarching mpm3DSimulation = selectedObject.GetComponent<Mpm3DMarching>();
                mpm3DSimulation.use_correct_cfl = toggle.isOn;
            }
        }
    }

    void OnDropdownValueChanged(TMP_Dropdown dropdown, int value)
    {
        Debug.Log(dropdown.name + " selected: " + dropdown.options[value].text);

        if (dropdown.name == "Dropdown_PrimitiveShape")
        {
            // Select a primitive shape and adjust the parameters
            if (dropdown.options[value].text != "Create a Primitive Shape")
            {
                prefabName = dropdown.options[value].text;
            }
            // Only adjust parameters here
            if (prefabName != null)
            {
                if (prefabName == "Sphere")
                {
                    ShapeParameterObject_1.SetActive(true);
                    TMP_Text parameter_text = ShapeParameterObject_1.transform.Find("Name").GetComponent<TMP_Text>();
                    parameter_text.text = "Radius";
                    ShapeParameterObject_1.GetComponentInChildren<Slider>().value = 0.5f;
                }
                if (prefabName == "Cube")
                {
                    ShapeParameterObject_1.SetActive(true);
                    TMP_Text parameter_text = ShapeParameterObject_1.transform.Find("Name").GetComponent<TMP_Text>();
                    parameter_text.text = "Size";
                    ShapeParameterObject_1.GetComponentInChildren<Slider>().value = 0.5f;
                }
                if (prefabName == "Cylinder")
                {
                    ShapeParameterObject_1.SetActive(true);
                    TMP_Text parameter_text_1 = ShapeParameterObject_1.transform.Find("Name").GetComponent<TMP_Text>();
                    parameter_text_1.text = "Cylinder Length";
                    ShapeParameterObject_1.GetComponentInChildren<Slider>().value = 0.8f;
                    ShapeParameterObject_2.SetActive(true);
                    TMP_Text parameter_text_2 = ShapeParameterObject_2.transform.Find("Name").GetComponent<TMP_Text>();
                    parameter_text_2.text = "Cylinder Radius";
                    ShapeParameterObject_2.GetComponentInChildren<Slider>().value = 0.05f;
                }
                if (prefabName == "Torus")
                {
                    ShapeParameterObject_1.SetActive(true);
                    TMP_Text parameter_text_1 = ShapeParameterObject_1.transform.Find("Name").GetComponent<TMP_Text>();
                    parameter_text_1.text = "Torus Radius";
                    ShapeParameterObject_1.GetComponentInChildren<Slider>().value = 0.3f;
                    ShapeParameterObject_2.SetActive(true);
                    TMP_Text parameter_text_2 = ShapeParameterObject_2.transform.Find("Name").GetComponent<TMP_Text>();
                    parameter_text_2.text = "Torus Tube Radius";
                    ShapeParameterObject_2.GetComponentInChildren<Slider>().value = 0.05f;
                }
            }
            // CreateMpm3DObjectFromPrefab(); 
            dropdown.value = 0;
            dropdown.Hide();
        }
        if (dropdown.name == "Dropdown_Color")
        {
            // Adjust the color of the primitive shape
            if (selectedObject != null)
            {
                Mpm3DMarching mpm3DSimulation = selectedObject.GetComponent<Mpm3DMarching>();
                if (mpm3DSimulation != null)
                {
                    if (dropdown.options[value].text == "Red")
                    {
                        mpm3DSimulation.AdjustTextureColor(Color.red);
                    }
                    else if (dropdown.options[value].text == "Orange")
                    {
                        mpm3DSimulation.AdjustTextureColor(new Color(1.0f, 0.5f, 0.0f));
                    }
                    else if (dropdown.options[value].text == "Green")
                    {
                        mpm3DSimulation.AdjustTextureColor(Color.green);
                    }
                    else if (dropdown.options[value].text == "Blue")
                    {
                        mpm3DSimulation.AdjustTextureColor(Color.blue);
                    }
                    else if (dropdown.options[value].text == "Yellow")
                    {
                        mpm3DSimulation.AdjustTextureColor(Color.yellow);
                    }
                    else if (dropdown.options[value].text == "White")
                    {
                        mpm3DSimulation.AdjustTextureColor(Color.white);
                    }
                    else if (dropdown.options[value].text == "Black")
                    {
                        mpm3DSimulation.AdjustTextureColor(Color.black);
                    }
                    else if (dropdown.options[value].text == "Brown")
                    {
                        mpm3DSimulation.AdjustTextureColor(new Color(0.6f, 0.3f, 0.0f));
                    }
                }
                dropdown.value = 0;
                dropdown.Hide();
            }
        }
        if (dropdown.name == "Dropdown_LeftHandTool")
        {
            // Select a tool for modeling used by the left hand
            if (selectedObject != null)
            {
                if (dropdown.options[value].text == "Left Hand Tool")
                {
                    SelectTools(selectedObject, "MatTool_Hand_Left", prevRightHandTool);
                }
                else if (dropdown.options[value].text == "Planar Pad")
                {
                    SelectTools(selectedObject, "MatTool_Pad_Left", prevRightHandTool);
                }
                else if (dropdown.options[value].text == "Rod")
                {
                    SelectTools(selectedObject, "MatTool_Rod_Left", prevRightHandTool);
                }
                else if (dropdown.options[value].text == "Cone")
                {
                    SelectTools(selectedObject, "MatTool_Cone_Left", prevRightHandTool);
                }
                else if (dropdown.options[value].text == "Slab")
                {
                    SelectTools(selectedObject, "MatTool_Slab_Left", prevRightHandTool);
                }
                else if (dropdown.options[value].text == "Scissor")
                {
                    SelectTools(selectedObject, "MatTool_Scissor_Left", prevRightHandTool);
                }
            }
        }
        if (dropdown.name == "Dropdown_RightHandTool")
        {
            // Select a tool for modeling used by the right hand
            if (selectedObject != null)
            {
                if (dropdown.options[value].text == "Right Hand Tool")
                {
                    SelectTools(selectedObject, prevLeftHandTool, "MatTool_Hand_Right");
                }
                else if (dropdown.options[value].text == "Planar Pad")
                {
                    SelectTools(selectedObject, prevLeftHandTool, "MatTool_Pad_Right");
                }
                else if (dropdown.options[value].text == "Rod")
                {
                    SelectTools(selectedObject, prevLeftHandTool, "MatTool_Rod_Right");
                }
                else if (dropdown.options[value].text == "Cone")
                {
                    SelectTools(selectedObject, prevLeftHandTool, "MatTool_Cone_Right");
                }
                else if (dropdown.options[value].text == "Slab")
                {
                    SelectTools(selectedObject, prevLeftHandTool, "MatTool_Slab_Right");
                }
                else if (dropdown.options[value].text == "Scissor")
                {
                    SelectTools(selectedObject, prevLeftHandTool, "MatTool_Scissor_Right");
                }
            }
        }
        if (dropdown.name == "Dropdown_MaterialType")
        {
            // if (selectedObject != null)
            // {
            //     var mpm3DSimulation = selectedObject.GetComponent<Mpm3DMarching>();
            //     if (dropdown.options[value].text == "Dough")
            //     {
            //         mpm3DSimulation._SigY = 0.0f;
            //         mpm3DSimulation._E = 1e7f;
            //         mpm3DSimulation.stressType = Mpm3DMarching.StressType.NeoHookean;
            //         mpm3DSimulation.materialType = Mpm3DMarching.MaterialType.Dough;
            //         mpm3DSimulation.damping = 200f;
            //         mpm3DSimulation.plasticityType = Mpm3DMarching.PlasticityType.Von_Mises;
            //         mpm3DSimulation.Init_materials();
            //         mpm3DSimulation.Update_materials();
            //     }
            //     else if (dropdown.options[value].text == "Clay")
            //     {
            //         mpm3DSimulation._SigY = 1e6f;
            //         mpm3DSimulation._E = 1e7f;
            //         mpm3DSimulation.stressType = Mpm3DMarching.StressType.NeoHookean;
            //         mpm3DSimulation.materialType = Mpm3DMarching.MaterialType.Dough;
            //         mpm3DSimulation.damping = 200f;
            //         mpm3DSimulation.plasticityType = Mpm3DMarching.PlasticityType.Von_Mises;
            //         mpm3DSimulation.Init_materials();
            //         mpm3DSimulation.Update_materials();
            //     }
            //     else if (dropdown.options[value].text == "Elastic_Material")
            //     {
            //         mpm3DSimulation._SigY = 1e7f;
            //         mpm3DSimulation._E = 1e7f;
            //         mpm3DSimulation.stressType = Mpm3DMarching.StressType.NeoHookean;
            //         mpm3DSimulation.materialType = Mpm3DMarching.MaterialType.Dough;
            //         mpm3DSimulation.damping = 200f;
            //         mpm3DSimulation.plasticityType = Mpm3DMarching.PlasticityType.Elastic;
            //         mpm3DSimulation.Init_materials();
            //         mpm3DSimulation.Update_materials();
            //     }
            // }
        }
        if (dropdown.name == "Dropdown_PlasticityType")
        {
        }
        if (dropdown.name == "Dropdown_StressType")
        {
        }
    }

    void CreateMpm3DObjectFromPrefab()
    {
        GameObject Mpm3DObject = Resources.Load<GameObject>("Prefabs/PrimitiveShapes/Mpm3DExample");

        if (Mpm3DObject != null)
        {
            // Position and name
            Vector3 position = sceneCamera.transform.position + sceneCamera.transform.forward * 0.1f;
            position.x -= 0.2f;
            position.y -= 0.2f;
            position.z += 0.2f;
            Quaternion rotation = Quaternion.LookRotation(sceneCamera.transform.forward);

            GameObject newMpm3DObject = Instantiate(Mpm3DObject, position, rotation);
            createdObjectLists.Add(newMpm3DObject);

            // Use the just created object as the selected object for further interactions
            selectedObject = newMpm3DObject;
            newMpm3DObject.name = "Mpm3DObject_" + createdObjectLists.Count;

            // Initialize the object
            Mpm3DMarching mpm3DSimulation = newMpm3DObject.GetComponent<Mpm3DMarching>();
            if (prefabName == "Sphere")
            {
                mpm3DSimulation.initShape = Mpm3DMarching.InitShape.Sphere;
                mpm3DSimulation.cube_size = ShapeParameterObject_1.GetComponentInChildren<Slider>().value;
            }
            if (prefabName == "Cube")
            {
                mpm3DSimulation.initShape = Mpm3DMarching.InitShape.Cube;
                mpm3DSimulation.cube_size = ShapeParameterObject_1.GetComponentInChildren<Slider>().value;
            }
            if (prefabName == "Cylinder")
            {
                mpm3DSimulation.initShape = Mpm3DMarching.InitShape.Cylinder;
                mpm3DSimulation.cylinder_length = ShapeParameterObject_1.GetComponentInChildren<Slider>().value;
                mpm3DSimulation.cylinder_radius = ShapeParameterObject_2.GetComponentInChildren<Slider>().value;
            }
            if (prefabName == "Torus")
            {
                mpm3DSimulation.initShape = Mpm3DMarching.InitShape.Torus;
                mpm3DSimulation.torus_radius = ShapeParameterObject_1.GetComponentInChildren<Slider>().value;
                mpm3DSimulation.torus_tube_radius = ShapeParameterObject_2.GetComponentInChildren<Slider>().value;
            }

            mpm3DSimulation.Initiate(); // Initialization
            SelectTools(selectedObject, prevLeftHandTool, prevRightHandTool); // Select tools for modeling
            mpm3DSimulation.leftPinchGesture = pinchGestureLeft; // Assign pinch gesture
            mpm3DSimulation.rightPinchGesture = pinchGestureRight; // Assign pinch gesture 

            // ApplyMaterial(newMpm3DObject); // Apply materials specified from UI
        }
    }

    void OnInputFieldSelect(InputField inputField)
    {
        Debug.Log(inputField.name + " was selected!");
        overlayKeyboard = TouchScreenKeyboard.Open(inputField.text, TouchScreenKeyboardType.Default);
    }

    void OnTMPInputFieldSelect(TMP_InputField inputField)
    {
        Debug.Log(inputField.name + " was selected!");
        overlayKeyboard = TouchScreenKeyboard.Open(inputField.text, TouchScreenKeyboardType.ASCIICapable);
    }

    public void HideCanvas()
    {
        if (UI_canvas != null)
        {
            UI_canvas.SetActive(false);
        }
    }

    public void ShowCanvas()
    {
        if (UI_canvas != null && oculus_hands[1].IsTracked && sceneCamera != null)
        {
            // Set position and rotation of the UI canvas
            Vector3 handThumbTipPosition = Vector3.zero;

            foreach (var b in oculus_skeletons[1].Bones)
            {
                if (b.Id == OVRSkeleton.BoneId.Hand_ThumbTip)
                {
                    handThumbTipPosition = b.Transform.position;
                    break;
                }
            }
            UI_canvas.SetActive(true);
            UI_anchor.position = handThumbTipPosition + sceneCamera.transform.forward * 0.2f;
            UI_anchor.rotation = Quaternion.LookRotation(sceneCamera.transform.forward);
            UI_canvas.transform.position = UI_anchor.position + canvas_anchor_offset;
            UI_canvas.transform.rotation = UI_anchor.rotation;
        }
    }

    public void ShowSelectedObjectCanvas()
    {
        if (UI_canvas != null && sceneCamera != null)
        {
            // Open the UI canvas for the object just grabbed
            if (selectedObject != null)
            {
                Mpm3DMarching mpm3DSimulation = selectedObject.GetComponent<Mpm3DMarching>();
                if (mpm3DSimulation != null)
                {
                    // Update the grid size number, rendering smooth iterations in UI
                    string initial_text = valueAdjustGridSize.GetComponent<TMP_Text>().text;
                    int grid_size = mpm3DSimulation.GetGridSize(); // grid size
                    string new_text = initial_text.Substring(0, initial_text.IndexOf(":") + 2) + (grid_size).ToString();
                    valueAdjustGridSize.GetComponent<TMP_Text>().text = new_text;
                    string initial_text_smooth = valueAdjustSmoothness.GetComponent<TMP_Text>().text;
                    int smooth_value = mpm3DSimulation.GetSmoothingIterations(); // smooth iterations
                    string new_text_smooth = initial_text_smooth.Substring(0, initial_text_smooth.IndexOf(":") + 2) + (smooth_value).ToString();
                    valueAdjustSmoothness.GetComponent<TMP_Text>().text = new_text_smooth;

                    // Set the toggle, dropdown, and slider values accordingly
                    foreach (Toggle toggle in toggles)
                    {
                        if (toggle.name == "Toggle_FixObject")
                        {
                            toggle.isOn = mpm3DSimulation.GetIsFixed();
                        }
                        if (toggle.name == "Toggle_EnableInteraction")
                        {
                            toggle.isOn = mpm3DSimulation.RunSimulation;
                        }
                        if (toggle.name == "Toggle_EnablePinchGesture_Left")
                        {
                            toggle.isOn = mpm3DSimulation.UsePinchGestureLeft;
                        }
                        if (toggle.name == "Toggle_EnablePinchGesture_Right")
                        {
                            toggle.isOn = mpm3DSimulation.UsePinchGestureRight;
                        }
                        if (toggle.name == "Toggle_StickyGround")
                        {
                            toggle.isOn = mpm3DSimulation.GetIsStickyBoundary();
                        }
                        if (toggle.name == "Toggle_Gravity")
                        {
                            toggle.isOn = mpm3DSimulation.GetGravity() != 0.0f;
                        }
                        if (toggle.name == "Toggle_UseCorrectCFL")
                        {
                            toggle.isOn = mpm3DSimulation.use_correct_cfl;
                        }
                    }

                    foreach (TMP_Dropdown dropdown in dropdowns)
                    {
                        if (dropdown.name == "Dropdown_MaterialType")
                        {
                            dropdown.value = (int)mpm3DSimulation.materialType;
                        }
                        if (dropdown.name == "Dropdown_PlasticityType")
                        {
                            dropdown.value = (int)mpm3DSimulation.plasticityType;
                        }
                        if (dropdown.name == "Dropdown_StressType")
                        {
                            dropdown.value = (int)mpm3DSimulation.stressType;
                        }
                    }

                    foreach (GameObject parameter in parameterObjects)
                    {
                        if (parameter.name == "Parameter_Pinch_Selection_Radius")
                        {
                            parameter.GetComponentInChildren<Slider>().value = pinchGestureLeft.pinchRadius;
                        }
                        if (parameter.name == "Parameter_Pinch_Force")
                        {
                            parameter.GetComponentInChildren<Slider>().value = mpm3DSimulation.GetPinchForceRatio();
                        }
                        if (parameter.name == "Parameter_E")
                        {
                            parameter.GetComponentInChildren<Slider>().value = mpm3DSimulation._E;
                        }
                        if (parameter.name == "Parameter_SigY")
                        {
                            parameter.GetComponentInChildren<Slider>().value = mpm3DSimulation._SigY;
                        }
                        if (parameter.name == "Parameter_Nu")
                        {
                            parameter.GetComponentInChildren<Slider>().value = mpm3DSimulation._nu;
                        }
                        if (parameter.name == "Parameter_ColideFactor")
                        {
                            parameter.GetComponentInChildren<Slider>().value = mpm3DSimulation.colide_factor;
                        }
                        if (parameter.name == "Parameter_FrictionK")
                        {
                            parameter.GetComponentInChildren<Slider>().value = mpm3DSimulation.friction_k;
                        }
                        if (parameter.name == "Parameter_P_Rho")
                        {
                            parameter.GetComponentInChildren<Slider>().value = mpm3DSimulation.p_rho;
                        }
                        if (parameter.name == "Parameter_FrictionAngle")
                        {
                            parameter.GetComponentInChildren<Slider>().value = mpm3DSimulation.friction_angle;
                        }
                        if (parameter.name == "Parameter_Damping")
                        {
                            parameter.GetComponentInChildren<Slider>().value = mpm3DSimulation.damping;
                        }
                        if (parameter.name == "Parameter_nGrid")
                        {
                            parameter.GetComponentInChildren<Slider>().value = (float)mpm3DSimulation.n_grid;
                        }
                        if (parameter.name == "Parameter_ParticlePerGrid")
                        {
                            parameter.GetComponentInChildren<Slider>().value = mpm3DSimulation.particle_per_grid;
                        }
                        if (parameter.name == "Parameter_MaxDt")
                        {
                            parameter.GetComponentInChildren<Slider>().value = mpm3DSimulation.max_dt;
                        }
                        if (parameter.name == "Parameter_FrameTime")
                        {
                            parameter.GetComponentInChildren<Slider>().value = mpm3DSimulation.frame_time;
                        }
                        if (parameter.name == "Parameter_AllowedCFL")
                        {
                            parameter.GetComponentInChildren<Slider>().value = mpm3DSimulation.allowed_cfl;
                        }
                        if (parameter.name == "Parameter_HandSimulationRadius")
                        {
                            parameter.GetComponentInChildren<Slider>().value = mpm3DSimulation.GetHandsimulationRadius();
                        }
                    }
                }
            }

            // Set position and rotation of the UI canvas
            Vector3 handThumbTipPosition = Vector3.zero;
            foreach (var b in oculus_skeletons[1].Bones)
            {
                if (b.Id == OVRSkeleton.BoneId.Hand_ThumbTip)
                {
                    handThumbTipPosition = b.Transform.position;
                    break;
                }
            }
            UI_canvas.SetActive(true);

            // Move the UI canvas to the hand position
            // UI_anchor.position = handThumbTipPosition + sceneCamera.transform.forward * 0.3f; 
            // UI_anchor.rotation = Quaternion.LookRotation(sceneCamera.transform.forward);
            // UI_canvas.transform.position = UI_anchor.position + canvas_anchor_offset;
            // UI_canvas.transform.rotation = UI_anchor.rotation;
        }
    }

    void OnDestroy()
    {

    }
}

[System.Serializable]
public class ObjectParameters
{
    public enum MaterialType
    {
        Customize,
        Clay,
        Dough,
        Elastic_Material
    }
    public enum PlasticityType
    {
        Von_Mises,
        Drucker_Prager,
        Clamp,
        Elastic
    }

    public enum StressType
    {
        NeoHookean,
        Kirchhoff
    }
    public string name;
    public bool isFixed = false;
    public bool isInModelingMode = true;
    public MaterialType materialType = MaterialType.Customize;
    public PlasticityType plasticityType = PlasticityType.Von_Mises;
    public StressType stressType = StressType.NeoHookean;
    public float E = 1e6f;
    public float SigY = 1e5f;
    public float nu = 0.3f;
    public float colide_factor = 0.5f;
    public float friction_k = 0.4f;
    public float p_rho = 1000;
    public float min_clamp = 0.1f;
    public float max_clamp = 0.1f;
    public float friction_angle = 30;
}