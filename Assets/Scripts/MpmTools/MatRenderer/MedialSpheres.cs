using UnityEngine;

public class MedialSpheres : MonoBehaviour
{
    public MATCoordinator matCoordinator; // Riferimento alla classe MATCoordinator
    public GameObject spherePrefab; // Il prefab della sfera da utilizzare

    private void Start()
    {

    }

    public void GenerateMedialSpheres()
    {
        Vector3[] vertices = matCoordinator.Vertices; // Ottieni i vertici dalla classe MATCoordinator
        float[] radii = matCoordinator.Radii; // Ottieni i raggi dalla classe MATCoordinator

        if (vertices == null || radii == null || vertices.Length != radii.Length)
        {
            Debug.LogError("Dati dei vertici o dei raggi non validi.");
            return;
        }

        for (int i = 0; i < vertices.Length; i++)
        {
            Vector3 position = vertices[i];
            float radius = radii[i];

            GameObject sphere = Instantiate(spherePrefab, position, Quaternion.identity);
            sphere.transform.localScale = Vector3.one * radius * 2; // Imposta la scala in base al raggio
            sphere.transform.parent = transform;
        }
    }
}
