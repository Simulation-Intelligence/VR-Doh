using System.Collections;
using UnityEngine;

public class SubmeshController : MonoBehaviour
{
    public float delayBetweenBatches = 1.0f; // Time in seconds between revealing each batch of submeshes
    public int submeshesPerBatch = 2; // Number of submeshes to reveal per batch
    public bool showSpecificSubmeshesOnly = false; // If true, show only the specific submeshes in the array
    public int[] submeshIndexesToShow; // Array containing the submesh indexes to show when showSpecificSubmeshesOnly is true

    private MeshRenderer meshRenderer;
    private MeshFilter meshFilter;
    private Material[] originalMaterials;

    void Start()
    {
        meshRenderer = GetComponent<MeshRenderer>();
        meshFilter = GetComponent<MeshFilter>();

        if (meshRenderer != null && meshFilter != null)
        {
            // Store the original materials of the submeshes
            originalMaterials = meshRenderer.materials;

            // Initially hide all submeshes
            Material[] invisibleMaterials = new Material[originalMaterials.Length];
            for (int i = 0; i < invisibleMaterials.Length; i++)
            {
                Material mat = new Material(Shader.Find("Standard"));
                mat.color = new Color(0, 0, 0, 0); // Fully transparent
                mat.SetFloat("_Mode", 3); // Enable transparency mode
                mat.SetInt("_SrcBlend", (int)UnityEngine.Rendering.BlendMode.SrcAlpha);
                mat.SetInt("_DstBlend", (int)UnityEngine.Rendering.BlendMode.OneMinusSrcAlpha);
                mat.SetInt("_ZWrite", 0);
                mat.DisableKeyword("_ALPHATEST_ON");
                mat.EnableKeyword("_ALPHABLEND_ON");
                mat.DisableKeyword("_ALPHAPREMULTIPLY_ON");
                mat.renderQueue = 3000;

                invisibleMaterials[i] = mat;
            }

            meshRenderer.materials = invisibleMaterials;

            // Start revealing the submeshes
            if (showSpecificSubmeshesOnly)
            {
                StartCoroutine(RevealSpecificSubmeshes());
            }
            else
            {
                StartCoroutine(RevealSubmeshesInBatches());
            }
        }
    }

    // Coroutine to reveal specific submeshes defined in the array
    IEnumerator RevealSpecificSubmeshes()
    {
        int revealedCount = 0;
        while (revealedCount < submeshIndexesToShow.Length)
        {
            Material[] currentMaterials = meshRenderer.materials;
            int indexToShow = submeshIndexesToShow[revealedCount];
            if (indexToShow < originalMaterials.Length)
            {
                currentMaterials[indexToShow] = originalMaterials[indexToShow];
                revealedCount++;
            }

            // Apply the updated materials
            meshRenderer.materials = currentMaterials;

            // Wait for the next batch (optional, could be removed if you want to show them all instantly)
            yield return new WaitForSeconds(delayBetweenBatches);
        }
    }

    // Coroutine to reveal submeshes in batches
    IEnumerator RevealSubmeshesInBatches()
    {
        int revealedCount = 0;
        while (revealedCount < originalMaterials.Length)
        {
            // Reveal the next batch of submeshes
            Material[] currentMaterials = meshRenderer.materials;
            for (int i = 0; i < submeshesPerBatch; i++)
            {
                if (revealedCount < originalMaterials.Length)
                {
                    currentMaterials[revealedCount] = originalMaterials[revealedCount];
                    revealedCount++;
                }
            }

            // Apply the updated materials
            meshRenderer.materials = currentMaterials;

            // Wait for the next batch
            yield return new WaitForSeconds(delayBetweenBatches);
        }
    }
}
