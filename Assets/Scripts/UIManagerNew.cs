using System;
using System.IO;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.UI;
using TMPro;
using System.Text;
using Oculus.Interaction;
using System.Linq;
class UIManagerNew : MonoBehaviour
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
    public Material SimulationBoxMaterial;
    public Material HighlightedSimulationBoxMaterial;

    [SerializeField]
    private GameObject colorPickerObject;
    private ColorPicker colorPicker;
    public GameObject ShapeParameterObject_1, ShapeParameterObject_2;
    public GameObject PinchParameterObject_1, PinchParameterObject_2;
    public GameObject RightSidePanel;
    public GameObject bk1, bk2, bk3, bk4;

    private List<GameObject> createdObjectLists = new List<GameObject>();
    private List<GameObject> removedObjectLists = new List<GameObject>(); // Merged objects are removed
    private GameObject selectedObject;
    private GameObject prevSelectedObject;
    private string prefabName;
    public string export_folder_path;
    private bool EnableObjectGrab = true;

    // UI Components
    public bool FixUIPosition = false;
    public GameObject UI_canvas;
    private Transform UI_panel;
    public Transform UI_anchor;
    private Vector3 canvas_anchor_offset;
    [SerializeField]
    private Camera sceneCamera;
    [SerializeField]
    private OVRHand[] oculus_hands;
    [SerializeField]
    private OVRSkeleton[] oculus_skeletons;
    public GameObject mergePrompt;
    public GameObject valueAdjustGridSize;
    public GameObject valueAdjustRenderGridSize;
    public GameObject valueAdjustSmoothness;
    public GameObject AdjustSmoothButtons;
    private bool isMerging = false;
    public Button[] buttons;
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
        UI_panel = UI_canvas.transform.Find("Unity Canvas");

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

        // Update shape slider value in UI
        TMP_Text shape_parameter_1_text = ShapeParameterObject_1.transform.Find("Name").GetComponent<TMP_Text>();
        TMP_Text shape_parameter_2_text = ShapeParameterObject_2.transform.Find("Name").GetComponent<TMP_Text>();
        Slider shape_parameter_1_slider = ShapeParameterObject_1.GetComponentInChildren<Slider>();
        Slider shape_parameter_2_slider = ShapeParameterObject_2.GetComponentInChildren<Slider>();
        string initial_text_2 = shape_parameter_2_text.text;
        shape_parameter_1_slider.onValueChanged.AddListener((float value) =>
        {
            string initial_text = shape_parameter_1_text.text;
            string initial_text_1 = initial_text.Substring(0, initial_text.IndexOf(":") + 2);
            shape_parameter_1_text.text = initial_text_1 + value.ToString("F2");
            shape_parameter_1_slider.value = (float)Math.Round(value, 2);
        });
        shape_parameter_2_slider.onValueChanged.AddListener((float value) =>
        {
            string initial_text = shape_parameter_2_text.text;
            string initial_text_2 = initial_text.Substring(0, initial_text.IndexOf(":") + 2);
            shape_parameter_2_text.text = initial_text_2 + value.ToString("F2");
            shape_parameter_2_slider.value = (float)Math.Round(value, 2);
        });
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
            ShowSelectedObjectInfo();
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
                                isMerging = false;
                                mpm3DSimulation.MergeAndUpdate(objectToMerge.GetComponent<Mpm3DMarching>());
                                Debug.Log("Merge object " + selectedObject.name + " with object " + objectToMerge.name);
                                // mergePrompt.SetActive(false);
                                // removedObjectLists.Add(objectToMerge); // Need to distroy the merged object
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
                            HighlightSelectedObject();
                            ShowSelectedObjectInfo();
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

                // Use only the right hand for squeezing
                pinchGestureRight.RenderSqueezeSphere = selectedObject.GetComponent<Mpm3DMarching>().squeeze_particles;
                pinchGestureRight.RenderSqueezeCone = selectedObject.GetComponent<Mpm3DMarching>().squeeze_particles;

                // Disable object grab when pinch gesture is enabled, avoiding unexpected rotation
                if (EnableObjectGrab)
                {
                    if ((pinchGestureLeft.RenderPinchSphere && pinchGestureLeft.isPinching) || (pinchGestureRight.RenderPinchSphere && pinchGestureRight.isPinching)
                     || (pinchGestureLeft.RenderRotationSphere && pinchGestureLeft.isRotating) || (pinchGestureRight.RenderRotationSphere && pinchGestureRight.isRotating)
                     || (pinchGestureRight.RenderSqueezeSphere && pinchGestureRight.RenderSqueezeCone && pinchGestureRight.isSqueezing))
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

    void HighlightSelectedObject()
    {
        if (prevSelectedObject != null && prevSelectedObject.transform.Find("Visuals") != null)
        {
            Transform VisualObject = prevSelectedObject.transform.Find("Visuals");
            MeshRenderer meshRenderer = VisualObject.Find("Mesh").GetComponent<MeshRenderer>();
            meshRenderer.material = SimulationBoxMaterial;
        }
        if (selectedObject != null && selectedObject.transform.Find("Visuals") != null)
        {
            Transform VisualObject = selectedObject.transform.Find("Visuals");
            MeshRenderer meshRenderer = VisualObject.Find("Mesh").GetComponent<MeshRenderer>();
            meshRenderer.material = HighlightedSimulationBoxMaterial;
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
        if (button.GetComponentInChildren<TMP_Text>() != null)
        {
            if (button.GetComponentInChildren<TMP_Text>().text == "Apply")
            {
                button.GetComponentInChildren<TMP_Text>().text = "Applyed!";
            }
            else if (button.GetComponentInChildren<TMP_Text>().text == "Applyed!")
            {
                button.GetComponentInChildren<TMP_Text>().text = "Apply";
            }
        }

        // Create a new object in the scene
        if (button.name == "Button_CreateObject")
        {
            // CreateNewMpm3DObject();
        }
        if (button.name == "SphereShape")
        {
            ShapeParameterObject_1.SetActive(true);
            ShapeParameterObject_1.GetComponentInChildren<Slider>().value = 0.5f;
            TMP_Text parameter_text = ShapeParameterObject_1.transform.Find("Name").GetComponent<TMP_Text>();
            parameter_text.text = "Radius: " + ShapeParameterObject_1.GetComponentInChildren<Slider>().value.ToString("F2");
            prefabName = "Sphere";

            // Adjust UI background
            RectTransform rectTransform_1 = bk1.GetComponent<RectTransform>();
            Vector2 sizeDelta = rectTransform_1.sizeDelta;
            if (sizeDelta.y == 500)
            {
                sizeDelta.y += 140;
                rectTransform_1.sizeDelta = sizeDelta;
                RectTransform rectTransform_2 = bk2.GetComponent<RectTransform>();
                rectTransform_2.anchoredPosition = new Vector2(rectTransform_2.anchoredPosition.x, rectTransform_2.anchoredPosition.y - 140);
            }
        }
        if (button.name == "CubeShape")
        {
            ShapeParameterObject_1.SetActive(true);
            ShapeParameterObject_1.GetComponentInChildren<Slider>().value = 0.5f;
            TMP_Text parameter_text = ShapeParameterObject_1.transform.Find("Name").GetComponent<TMP_Text>();
            parameter_text.text = "Size: " + ShapeParameterObject_1.GetComponentInChildren<Slider>().value.ToString("F2");
            prefabName = "Cube";

            // Adjust UI background
            RectTransform rectTransform_1 = bk1.GetComponent<RectTransform>();
            Vector2 sizeDelta = rectTransform_1.sizeDelta;
            if (sizeDelta.y == 500)
            {
                sizeDelta.y += 140;
                rectTransform_1.sizeDelta = sizeDelta;
                RectTransform rectTransform_2 = bk2.GetComponent<RectTransform>();
                rectTransform_2.anchoredPosition = new Vector2(rectTransform_2.anchoredPosition.x, rectTransform_2.anchoredPosition.y - 140);
            }
        }
        if (button.name == "CylinderShape")
        {
            ShapeParameterObject_1.SetActive(true);
            ShapeParameterObject_1.GetComponentInChildren<Slider>().value = 0.8f;
            TMP_Text parameter_text_1 = ShapeParameterObject_1.transform.Find("Name").GetComponent<TMP_Text>();
            parameter_text_1.text = "Cylinder Length: " + ShapeParameterObject_1.GetComponentInChildren<Slider>().value.ToString("F2");
            ShapeParameterObject_2.SetActive(true);
            ShapeParameterObject_2.GetComponentInChildren<Slider>().value = 0.05f;
            TMP_Text parameter_text_2 = ShapeParameterObject_2.transform.Find("Name").GetComponent<TMP_Text>();
            parameter_text_2.text = "Cylinder Radius: " + ShapeParameterObject_2.GetComponentInChildren<Slider>().value.ToString("F2");
            prefabName = "Cylinder";

            // Adjust UI background
            RectTransform rectTransform_1 = bk1.GetComponent<RectTransform>();
            Vector2 sizeDelta = rectTransform_1.sizeDelta;
            if (sizeDelta.y == 500)
            {
                sizeDelta.y += 280;
                rectTransform_1.sizeDelta = sizeDelta;
                RectTransform rectTransform_2 = bk2.GetComponent<RectTransform>();
                rectTransform_2.anchoredPosition = new Vector2(rectTransform_2.anchoredPosition.x, rectTransform_2.anchoredPosition.y - 280);
            }
        }
        if (button.name == "TorusShape")
        {
            ShapeParameterObject_1.SetActive(true);
            ShapeParameterObject_1.GetComponentInChildren<Slider>().value = 0.22f;
            TMP_Text parameter_text_1 = ShapeParameterObject_1.transform.Find("Name").GetComponent<TMP_Text>();
            parameter_text_1.text = "Torus Radius: " + ShapeParameterObject_1.GetComponentInChildren<Slider>().value.ToString("F2");
            ShapeParameterObject_2.SetActive(true);
            ShapeParameterObject_2.GetComponentInChildren<Slider>().value = 0.08f;
            TMP_Text parameter_text_2 = ShapeParameterObject_2.transform.Find("Name").GetComponent<TMP_Text>();
            parameter_text_2.text = "Torus Tube Radius: " + ShapeParameterObject_2.GetComponentInChildren<Slider>().value.ToString("F2");
            prefabName = "Torus";

            // Adjust UI background
            RectTransform rectTransform_1 = bk1.GetComponent<RectTransform>();
            Vector2 sizeDelta = rectTransform_1.sizeDelta;
            if (sizeDelta.y == 500)
            {
                sizeDelta.y += 280;
                rectTransform_1.sizeDelta = sizeDelta;
                RectTransform rectTransform_2 = bk2.GetComponent<RectTransform>();
                rectTransform_2.anchoredPosition = new Vector2(rectTransform_2.anchoredPosition.x, rectTransform_2.anchoredPosition.y - 280);
            }
        }

        // Create object based on shape parameters
        if (button.name == "Button_Confirm")
        {
            ShapeParameterObject_1.SetActive(false);
            ShapeParameterObject_2.SetActive(false);
            CreateMpm3DObjectFromPrefab();

            // Adjust UI background
            RectTransform rectTransform_1 = bk1.GetComponent<RectTransform>();
            Vector2 sizeDelta = rectTransform_1.sizeDelta;
            sizeDelta.y = 500;
            rectTransform_1.sizeDelta = sizeDelta;
            RectTransform rectTransform_2 = bk2.GetComponent<RectTransform>();
            rectTransform_2.anchoredPosition = new Vector2(rectTransform_2.anchoredPosition.x, -740);
        }
        // Merge the object with another object
        if (button.name == "Button_MergeObject")
        {
            if (selectedObject != null)
            {
                if (button.GetComponentInChildren<TMP_Text>().text == "Merge")
                {
                    button.GetComponentInChildren<TMP_Text>().text = "Cancel Merging";
                    // mergePrompt.SetActive(true);
                    // mergePrompt.GetComponent<TMP_Text>().text = "Please select an object to merge";
                    isMerging = true;
                }
                else
                {
                    button.GetComponentInChildren<TMP_Text>().text = "Merge";
                    // mergePrompt.SetActive(false);
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
            }
        }
        // Recenter the object with original parameters
        if (button.name == "Button_RecenterObject")
        {
            if (selectedObject != null)
            {
                // Recenter the object to the visual box
                Mpm3DMarching mpm3DSimulation = selectedObject.GetComponent<Mpm3DMarching>();
                mpm3DSimulation.RecenterObject();
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
                    if (parameter.name == "Parameter_Pinch_Force_Ratio")
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
                selectedObject.GetComponent<Mpm3DMarching>().Dispose();
                Destroy(selectedObject);
                selectedObject = null;
            }
        }

        // Export marching cubes data to a file
        if (button.name == "Button_ExportObject")
        {
            if (export_folder_path != null)
            {
                if (selectedObject != null)
                {
                    Mpm3DMarching mpm3DSimulation = selectedObject.GetComponent<Mpm3DMarching>();
                    string export_file_path = export_folder_path + "/" + selectedObject.name + ".txt";
                    mpm3DSimulation.ExportData(export_file_path);
                    ExportMpm3DMesh(mpm3DSimulation);

                    // Show file name in UI
                    foreach (Button _button in buttons)
                    {
                        if (_button.name == "Button_ExportObject")
                        {
                            string initialText = _button.GetComponentInChildren<TMP_Text>().text;
                            if (initialText == "Export My Creation")
                            {
                                _button.GetComponentInChildren<TMP_Text>().text = "Exported!";
                            }
                            else
                            {
                                _button.GetComponentInChildren<TMP_Text>().text = "Export My Creation";
                            }
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
                        _button.GetComponentInChildren<TMP_Text>().text = "No Export Folder";
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
                string initial_text_smooth = valueAdjustSmoothness.GetComponent<Text>().text;
                int smooth_value = mpm3DSimulation.GetSmoothingIterations(); // smooth iterations
                string new_text_smooth = initial_text_smooth.Substring(0, initial_text_smooth.IndexOf(":") + 2) + (smooth_value).ToString();
                valueAdjustSmoothness.GetComponent<Text>().text = new_text_smooth;
            }
        }
        if (button.name == "Button_LessSmooth")
        {
            if (selectedObject != null)
            {
                Mpm3DMarching mpm3DSimulation = selectedObject.GetComponent<Mpm3DMarching>();
                mpm3DSimulation.DecreaseSmoothingIterations();
                string initial_text_smooth = valueAdjustSmoothness.GetComponent<Text>().text;
                int smooth_value = mpm3DSimulation.GetSmoothingIterations(); // smooth iterations
                string new_text_smooth = initial_text_smooth.Substring(0, initial_text_smooth.IndexOf(":") + 2) + (smooth_value).ToString();
                valueAdjustSmoothness.GetComponent<Text>().text = new_text_smooth;
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
        if (button.name == "Button_IncreaseRenderGridSize")
        {
            if (selectedObject != null)
            {
                int increaseValue = 8;
                Mpm3DMarching mpm3DSimulation = selectedObject.GetComponent<Mpm3DMarching>();
                mpm3DSimulation.IncreaseRenderGridSize(increaseValue);
                string initial_text = valueAdjustRenderGridSize.GetComponent<TMP_Text>().text;
                string new_text = initial_text.Substring(0, initial_text.IndexOf(":") + 2) + mpm3DSimulation.GetRenderGridSize().ToString();
                valueAdjustRenderGridSize.GetComponent<TMP_Text>().text = new_text;
            }
        }
        if (button.name == "Button_DecreaseRenderGridSize")
        {
            if (selectedObject != null)
            {
                int decreaseValue = 8;
                Mpm3DMarching mpm3DSimulation = selectedObject.GetComponent<Mpm3DMarching>();
                mpm3DSimulation.DecreaseRenderGridSize(decreaseValue);
                string initial_text = valueAdjustRenderGridSize.GetComponent<TMP_Text>().text;
                string new_text = initial_text.Substring(0, initial_text.IndexOf(":") + 2) + mpm3DSimulation.GetRenderGridSize().ToString();
                valueAdjustRenderGridSize.GetComponent<TMP_Text>().text = new_text;
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

        if (toggle.name == "Toggle_MoreOptions")
        {
            RectTransform rectTransform = UI_panel.GetComponent<RectTransform>();
            if (rectTransform != null)
            {
                if (isOn)
                {
                    Vector2 sizeDelta = rectTransform.sizeDelta;
                    sizeDelta.x = 2700;
                    rectTransform.sizeDelta = sizeDelta;
                    RightSidePanel.SetActive(true);
                }
                else
                {
                    Vector2 sizeDelta = rectTransform.sizeDelta;
                    sizeDelta.x = 1800;
                    rectTransform.sizeDelta = sizeDelta;
                    RightSidePanel.SetActive(false);
                }
            }
        }

        // Enable/Disable pinch gesture for mid-air interactions
        if (toggle.name == "Toggle_EnablePinchGesture_Left")
        {
            if (!isOn)
            {
                foreach (Toggle tog in toggles)
                {
                    if (tog.name == "Toggle_EnablePinchGesture_Right")
                    {
                        if (!tog.isOn)
                        {
                            PinchParameterObject_1.SetActive(false);
                            PinchParameterObject_2.SetActive(false);
                            // Adjust UI background
                            RectTransform rectTransform_3 = bk3.GetComponent<RectTransform>();
                            Vector2 sizeDelta = rectTransform_3.sizeDelta;
                            sizeDelta.y = 330;
                            rectTransform_3.sizeDelta = sizeDelta;
                            RectTransform rectTransform_4 = bk4.GetComponent<RectTransform>();
                            rectTransform_4.anchoredPosition = new Vector2(rectTransform_4.anchoredPosition.x, -570);
                        }
                    }
                }
            }
            else
            {
                if (!PinchParameterObject_1.activeSelf)
                {
                    PinchParameterObject_1.SetActive(true);
                    PinchParameterObject_2.SetActive(true);
                    // Adjust UI background
                    RectTransform rectTransform_3 = bk3.GetComponent<RectTransform>();
                    Vector2 sizeDelta = rectTransform_3.sizeDelta;
                    sizeDelta.y += 300;
                    rectTransform_3.sizeDelta = sizeDelta;
                    RectTransform rectTransform_4 = bk4.GetComponent<RectTransform>();
                    rectTransform_4.anchoredPosition = new Vector2(rectTransform_4.anchoredPosition.x, rectTransform_4.anchoredPosition.y - 300);
                }
            }
            if (selectedObject != null)
            {
                Mpm3DMarching mpm3DSimulation = selectedObject.GetComponent<Mpm3DMarching>();
                mpm3DSimulation.UsePinchGestureLeft = toggle.isOn;
            }
        }
        if (toggle.name == "Toggle_EnablePinchGesture_Right")
        {
            if (!isOn)
            {
                foreach (Toggle tog in toggles)
                {
                    if (tog.name == "Toggle_EnablePinchGesture_Left")
                    {
                        if (!tog.isOn)
                        {
                            PinchParameterObject_1.SetActive(false);
                            PinchParameterObject_2.SetActive(false);
                            // Adjust UI background
                            RectTransform rectTransform_3 = bk3.GetComponent<RectTransform>();
                            Vector2 sizeDelta = rectTransform_3.sizeDelta;
                            sizeDelta.y = 330;
                            rectTransform_3.sizeDelta = sizeDelta;
                            RectTransform rectTransform_4 = bk4.GetComponent<RectTransform>();
                            rectTransform_4.anchoredPosition = new Vector2(rectTransform_4.anchoredPosition.x, -570);
                        }
                    }
                }
            }
            else
            {
                if (!PinchParameterObject_1.activeSelf)
                {
                    PinchParameterObject_1.SetActive(true);
                    PinchParameterObject_2.SetActive(true);
                    // Adjust UI background
                    RectTransform rectTransform_3 = bk3.GetComponent<RectTransform>();
                    Vector2 sizeDelta = rectTransform_3.sizeDelta;
                    sizeDelta.y += 300;
                    rectTransform_3.sizeDelta = sizeDelta;
                    RectTransform rectTransform_4 = bk4.GetComponent<RectTransform>();
                    rectTransform_4.anchoredPosition = new Vector2(rectTransform_4.anchoredPosition.x, rectTransform_4.anchoredPosition.y - 300);
                }
            }
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
        if (toggle.name == "Toggle_EnabeSmoothButtons")
        {
            AdjustSmoothButtons.SetActive(isOn);
            // Adjust UI background
            if (isOn)
            {
                RectTransform rectTransform_1 = bk1.GetComponent<RectTransform>();
                Vector2 sizeDelta = rectTransform_1.sizeDelta;
                sizeDelta.y += 120;
                rectTransform_1.sizeDelta = sizeDelta;
                RectTransform rectTransform_2 = bk2.GetComponent<RectTransform>();
                rectTransform_2.anchoredPosition = new Vector2(rectTransform_2.anchoredPosition.x, rectTransform_2.anchoredPosition.y - 120);
            }
            else
            {
                RectTransform rectTransform_1 = bk1.GetComponent<RectTransform>();
                Vector2 sizeDelta = rectTransform_1.sizeDelta;
                sizeDelta.y -= 120;
                rectTransform_1.sizeDelta = sizeDelta;
                RectTransform rectTransform_2 = bk2.GetComponent<RectTransform>();
                rectTransform_2.anchoredPosition = new Vector2(rectTransform_2.anchoredPosition.x, rectTransform_2.anchoredPosition.y + 120);
            }
        }
        // Enable/Disable object grab
        if (toggle.name == "Toggle_EnableGrab")
        {
            if (selectedObject != null)
            {
                Text toggle_label = toggle.GetComponentInChildren<Text>();
                // toggle_label.text = isOn ? "Object is grabbable" : "Object is not grabbable";
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
                // toggle_label.text = isOn ? "Object is Interactable" : "Object is not Interactable";
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
        if (toggle.name == "Toggle_EnableGravity")
        {
            if (selectedObject != null)
            {
                Mpm3DMarching mpm3DSimulation = selectedObject.GetComponent<Mpm3DMarching>();
                mpm3DSimulation.SetGravity(isOn ? -200f : 0.0f);
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
        if (toggle.name == "Toggle_SquzzeParticles")
        {
            if (selectedObject != null)
            {
                Mpm3DMarching mpm3DSimulation = selectedObject.GetComponent<Mpm3DMarching>();
                mpm3DSimulation.squeeze_particles = toggle.isOn;
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
                    ShapeParameterObject_1.GetComponentInChildren<Slider>().value = 0.22f;
                    ShapeParameterObject_2.SetActive(true);
                    TMP_Text parameter_text_2 = ShapeParameterObject_2.transform.Find("Name").GetComponent<TMP_Text>();
                    parameter_text_2.text = "Torus Tube Radius";
                    ShapeParameterObject_2.GetComponentInChildren<Slider>().value = 0.08f;
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
        if (dropdown.name == "Dropdown_SqueezeType")
        {
            if (selectedObject != null)
            {
                Mpm3DMarching mpm3DSimulation = selectedObject.GetComponent<Mpm3DMarching>();
                if (dropdown.options[value].text == "Shapes (Circle)")
                {
                    mpm3DSimulation.squeezeType = Mpm3DMarching.SqueezeType.Circle;
                }
                else if (dropdown.options[value].text == "Star")
                {
                    mpm3DSimulation.squeezeType = Mpm3DMarching.SqueezeType.Star;
                }
                else if (dropdown.options[value].text == "Square")
                {
                    mpm3DSimulation.squeezeType = Mpm3DMarching.SqueezeType.Square;
                }
            }
        }
        if (dropdown.name == "Dropdown_MaterialType")
        {
            if (selectedObject != null)
            {
                Mpm3DMarching mpm3DSimulation = selectedObject.GetComponent<Mpm3DMarching>();
                if (dropdown.options[value].text == "Default Material (Clay)")
                {
                    mpm3DSimulation.plasticityType = Mpm3DMarching.PlasticityType.Von_Mises;
                    mpm3DSimulation._E = 1e7f;
                    mpm3DSimulation._SigY = 1e6f;
                    mpm3DSimulation.Init_materials();
                    mpm3DSimulation.Copy_materials();
                    mpm3DSimulation.materialType = Mpm3DMarching.MaterialType.Default_Clay;
                }
                else if (dropdown.options[value].text == "Soft Clay")
                {
                    mpm3DSimulation.plasticityType = Mpm3DMarching.PlasticityType.Von_Mises;
                    mpm3DSimulation._E = 1e7f;
                    mpm3DSimulation._SigY = 4e5f;
                    mpm3DSimulation.Init_materials();
                    mpm3DSimulation.Copy_materials();
                    mpm3DSimulation.materialType = Mpm3DMarching.MaterialType.Soft_Clay;
                }
                else if (dropdown.options[value].text == "Clamp Plasticity")
                {
                    mpm3DSimulation.plasticityType = Mpm3DMarching.PlasticityType.Clamp;
                    mpm3DSimulation._E = 1e7f;
                    mpm3DSimulation._SigY = 1e6f;
                    mpm3DSimulation.Init_materials();
                    mpm3DSimulation.Copy_materials();
                    mpm3DSimulation.materialType = Mpm3DMarching.MaterialType.Clamp_Plasticity;
                }
                else if (dropdown.options[value].text == "Drucker Plasticity")
                {
                    mpm3DSimulation.plasticityType = Mpm3DMarching.PlasticityType.Drucker_Prager;
                    mpm3DSimulation._E = 1e7f;
                    mpm3DSimulation._SigY = 1e6f;
                    mpm3DSimulation.Init_materials();
                    mpm3DSimulation.Copy_materials();
                    mpm3DSimulation.materialType = Mpm3DMarching.MaterialType.Drucker_Plasticity;
                }
                else if (dropdown.options[value].text == "Viscous Liquid")
                {
                    mpm3DSimulation.plasticityType = Mpm3DMarching.PlasticityType.Drucker_Prager;
                    mpm3DSimulation._E = 5e6f;
                    mpm3DSimulation._SigY = 1e6f;
                    mpm3DSimulation.Init_materials();
                    mpm3DSimulation.Copy_materials();
                    mpm3DSimulation.materialType = Mpm3DMarching.MaterialType.Viscous_Liquid;
                }
                ShowSelectedObjectInfo();
            }
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
            position.x -= 0.1f;
            position.y -= 0.3f;
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

        // Move the UI canvas according to the hand position
        if (!FixUIPosition)
        {
            UI_anchor.position = handThumbTipPosition + sceneCamera.transform.forward * 0.4f - sceneCamera.transform.up * 0.2f;
            UI_anchor.rotation = Quaternion.LookRotation(sceneCamera.transform.forward);
            UI_canvas.transform.position = UI_anchor.position + canvas_anchor_offset;
            UI_canvas.transform.rotation = UI_anchor.rotation;
        }
    }

    public void ShowSelectedObjectInfo()
    {
        if (UI_canvas != null && sceneCamera != null)
        {
            // Open the UI canvas for the object just grabbed
            if (selectedObject != null)
            {
                Mpm3DMarching mpm3DSimulation = selectedObject.GetComponent<Mpm3DMarching>();
                if (mpm3DSimulation != null)
                {
                    // Update the grid size number in UI
                    string initial_text = valueAdjustGridSize.GetComponent<TMP_Text>().text;
                    int grid_size = mpm3DSimulation.GetGridSize(); // grid size
                    string new_text = initial_text.Substring(0, initial_text.IndexOf(":") + 2) + (grid_size).ToString();
                    valueAdjustGridSize.GetComponent<TMP_Text>().text = new_text;

                    string initial_text_render = valueAdjustRenderGridSize.GetComponent<TMP_Text>().text;
                    int render_grid_size = mpm3DSimulation.GetRenderGridSize();
                    string new_text_render = initial_text_render.Substring(0, initial_text_render.IndexOf(":") + 2) + (render_grid_size).ToString();
                    valueAdjustRenderGridSize.GetComponent<TMP_Text>().text = new_text_render;

                    // Update rendering smooth iterations in UI
                    string initial_text_smooth = valueAdjustSmoothness.GetComponent<Text>().text;
                    int smooth_value = mpm3DSimulation.GetSmoothingIterations(); // smooth iterations
                    string new_text_smooth = initial_text_smooth.Substring(0, initial_text_smooth.IndexOf(":") + 2) + (smooth_value).ToString();
                    valueAdjustSmoothness.GetComponent<Text>().text = new_text_smooth;

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
                        if (toggle.name == "Toggle_SquzzeParticles")
                        {
                            toggle.isOn = mpm3DSimulation.squeeze_particles;
                        }
                        if (toggle.name == "Toggle_StickyGround")
                        {
                            toggle.isOn = mpm3DSimulation.GetIsStickyBoundary();
                        }
                        if (toggle.name == "Toggle_EnableGravity")
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
                        if (dropdown.name == "Dropdown_SqueezeType")
                        {
                            dropdown.value = (int)mpm3DSimulation.squeezeType;
                        }
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
                        if (parameter.name == "Parameter_Pinch_Force_Ratio")
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
                        if (parameter.name == "Parameter_CollideFactor")
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
        }
    }
    public void ExportMpm3DMesh(Mpm3DMarching mpm3DSimulation)
    {
        if (mpm3DSimulation != null)
        {
            GameObject obj = mpm3DSimulation.gameObject;
            string objPath = export_folder_path + "/" + obj.name + ".obj";
            string mtlPath = export_folder_path + "/" + obj.name + ".mtl";
            ExportMeshesToObj(obj, objPath, mtlPath);
        }
    }
    public void ExportMeshesToObj(GameObject obj, string objPath, string mtlPath)
    {
        // 使用输入的路径
        string exportObjPath = objPath ?? "Assets/ExportedMeshes.obj";  // 默认路径
        string exportMtlPath = mtlPath ?? "Assets/ExportedMaterials.mtl";  // 默认路径

        // 创建StringBuilder来保存整个.obj文件内容
        StringBuilder objContent = new StringBuilder();
        StringBuilder mtlContent = new StringBuilder();

        // 在.obj文件开头添加空行和mtllib
        objContent.AppendLine();  // 空行
        objContent.AppendLine($"mtllib {Path.GetFileName(mtlPath)}");  // 添加mtllib引用

        // 创建材质ID映射
        int materialIndex = 0;
        var materialToIndex = new System.Collections.Generic.Dictionary<Material, int>();

        // 初始化顶点偏移
        int vertexOffset = 1;

        // 遍历所有子物体
        MeshFilter[] meshFilters = obj.GetComponentsInChildren<MeshFilter>();
        foreach (MeshFilter meshFilter in meshFilters)
        {
            // 跳过名字不包含 "MarchingCubeVisualizer" 的子物体
            if (!meshFilter.gameObject.name.Contains("MarchingCubeVisualizer"))
            {
                continue; // 如果名字不符合条件，则跳过此物体
            }

            Mesh mesh = meshFilter.sharedMesh;
            if (mesh != null)
            {
                // 获取MeshRenderer组件的材质
                MeshRenderer meshRenderer = meshFilter.GetComponent<MeshRenderer>();
                if (meshRenderer != null)
                {
                    foreach (Material material in meshRenderer.sharedMaterials)
                    {
                        if (!materialToIndex.ContainsKey(material))
                        {
                            materialToIndex[material] = materialIndex++;

                            // 在.mtl文件中写入材质信息
                            mtlContent.AppendLine($"newmtl Material{materialToIndex[material]}");

                            // 设置漫反射颜色 (Kd) 和环境光颜色 (Ka)
                            mtlContent.AppendLine($"Kd {material.color.r} {material.color.g} {material.color.b}");
                            mtlContent.AppendLine($"Ka {material.color.r} {material.color.g} {material.color.b}");
                            mtlContent.AppendLine($"Ks 0 0 0");
                            mtlContent.AppendLine();
                        }

                        // 在.obj文件中指定使用哪个材质
                        objContent.AppendLine($"usemtl Material{materialToIndex[material]}");
                    }
                }

                // 从GPU中读取顶点和索引数据
                ReadGpuMeshData(meshFilter, objContent, ref vertexOffset);
            }
        }

        // 保存.obj文件
        File.WriteAllText(exportObjPath, objContent.ToString());
        // 保存.mtl文件
        File.WriteAllText(exportMtlPath, mtlContent.ToString());

        Debug.Log("Mesh and Material export completed: " + exportObjPath);
    }

    // 从GPU中读取顶点和索引数据
    void ReadGpuMeshData(MeshFilter meshFilter, StringBuilder objContent, ref int vertexOffset)
    {
        // 获取MeshRenderer组件的mesh
        Mesh mesh = meshFilter.sharedMesh;

        // 获取GPU上的顶点和索引缓冲区
        GraphicsBuffer vertexBuffer = mesh.GetVertexBuffer(0);  // 获取第一个顶点缓冲区
        GraphicsBuffer indexBuffer = mesh.GetIndexBuffer();     // 获取索引缓冲区

        // 读取整个顶点缓冲区的数据（24 字节每个顶点，包含位置和法线）
        VertexData[] vertexData = ReadVertexBuffer(vertexBuffer);

        // 获取索引数据
        int[] indices = ReadIndexBuffer(indexBuffer);

        // 根据最大索引确定需要多少个顶点
        int maxIndex = indices.Length > 0 ? indices.Max() : 0;
        int vertexCount = maxIndex + 1;

        // 截断索引数据 
        //如果索引出现连续3个0，则后面的索引数据都不需要了
        List<int> validIndices = new List<int>();
        for (int i = 0; i < indices.Length; i += 3)
        {
            if (indices[i] == 0 && indices[i + 1] == 0 && indices[i + 2] == 0)
            {
                break;
            }
            validIndices.Add(indices[i]);
            validIndices.Add(indices[i + 1]);
            validIndices.Add(indices[i + 2]);
        }

        // List<int> validIndices = new List<int>();
        // foreach (var index in indices)
        // {
        //     if (index == 0)
        //         break;
        //     validIndices.Add(index);
        // }

        // 当前Mesh的顶点编号，从vertexOffset开始
        int currentVertexOffset = vertexOffset;

        // 写入顶点数据（位置和法线）
        foreach (var data in vertexData.Take(vertexCount))
        {
            objContent.AppendLine($"v {data.position.x} {data.position.y} {data.position.z}");
            objContent.AppendLine($"vn {data.normal.x} {data.normal.y} {data.normal.z}");
        }

        // 写入有效的索引数据（面数据）
        for (int i = 0; i < validIndices.Count; i += 3)
        {
            if (i + 2 < validIndices.Count)
            {
                // 为当前mesh的顶点添加偏移
                objContent.AppendLine($"f {validIndices[i] + currentVertexOffset} {validIndices[i + 1] + currentVertexOffset} {validIndices[i + 2] + currentVertexOffset}");
            }
        }

        // 更新全局vertexOffset，累加当前mesh的顶点数量
        vertexOffset += vertexCount;
    }

    // 从GPU中的顶点缓冲区读取数据
    VertexData[] ReadVertexBuffer(GraphicsBuffer vertexBuffer)
    {
        // 读取整个顶点缓冲区数据（位置和法线）
        VertexData[] vertexData = new VertexData[vertexBuffer.count];  // 每个顶点包含位置和法线
        vertexBuffer.GetData(vertexData);
        return vertexData;
    }

    // 读取GPU中的索引数据
    int[] ReadIndexBuffer(GraphicsBuffer indexBuffer)
    {
        int[] indices = new int[indexBuffer.count];
        indexBuffer.GetData(indices);
        return indices;
    }

    // 顶点数据结构，包含位置和法线
    struct VertexData
    {
        public Vector3 position;
        public Vector3 normal;
    }
    void OnDestroy()
    {

    }
}