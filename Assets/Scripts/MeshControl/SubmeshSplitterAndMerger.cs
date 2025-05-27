using System.Collections;
using UnityEngine;

public class SubmeshSplitterAndMerger : MonoBehaviour
{
    public GameObject originalObject;  // 原始对象
    public float delayBetweenSubmeshes = 1.0f;  // 显示每个submesh的延迟

    void Start()
    {
        MeshFilter meshFilter = originalObject.GetComponent<MeshFilter>();
        if (meshFilter != null)
        {
            Mesh originalMesh = meshFilter.sharedMesh;
            SplitAndCreateSubmeshObjects(originalMesh);
        }
    }

    void SplitAndCreateSubmeshObjects(Mesh originalMesh)
    {
        // 获取submesh的数量
        int submeshCount = originalMesh.subMeshCount;
        // 获取原始材质
        Material[] originalMaterials = originalObject.GetComponent<MeshRenderer>().sharedMaterials;

        // 存储拆分出来的submesh对象
        GameObject[] submeshObjects = new GameObject[submeshCount];

        // 创建每个submesh的GameObject
        for (int i = 0; i < submeshCount; i++)
        {
            // 为每个submesh创建新的GameObject
            GameObject submeshObj = new GameObject("Submesh_" + i);
            submeshObjects[i] = submeshObj;

            // 添加MeshFilter和MeshRenderer组件
            MeshFilter submeshFilter = submeshObj.AddComponent<MeshFilter>();
            MeshRenderer submeshRenderer = submeshObj.AddComponent<MeshRenderer>();

            // 赋予submesh它自己的网格数据
            Mesh submesh = new Mesh();
            submesh.vertices = originalMesh.vertices;
            submesh.normals = originalMesh.normals;
            submesh.uv = originalMesh.uv;
            submesh.SetTriangles(originalMesh.GetTriangles(i), 0);

            // 将submesh应用到MeshFilter
            submeshFilter.mesh = submesh;

            // 使用原始对象的材质
            submeshRenderer.material = originalMaterials[i];

            // 调整位置，确保submesh不重叠
            submeshObj.transform.position = new Vector3(i * 2.0f, 0, 0);  // 简单示例：沿X轴偏移
        }

        // 启动一个协程，等待一段时间后合并submeshes
        StartCoroutine(MergeSubmeshes(submeshObjects, originalMaterials));
    }

    IEnumerator MergeSubmeshes(GameObject[] submeshObjects, Material[] originalMaterials)
    {
        // 等待一段时间让用户查看
        yield return new WaitForSeconds(delayBetweenSubmeshes * submeshObjects.Length);

        // 合并所有submesh
        CombineMeshes(submeshObjects, originalMaterials);
    }

    void CombineMeshes(GameObject[] submeshObjects, Material[] originalMaterials)
    {
        // 创建一个新的GameObject来存放合并后的Mesh
        GameObject combinedObject = new GameObject("CombinedMesh");
        MeshFilter combinedMeshFilter = combinedObject.AddComponent<MeshFilter>();
        MeshRenderer combinedMeshRenderer = combinedObject.AddComponent<MeshRenderer>();

        // 用来存储所有submesh的顶点、法线、纹理坐标等
        CombineInstance[] combine = new CombineInstance[submeshObjects.Length];
        int vertexOffset = 0;

        // 存储每个submesh的材质
        Material[] combinedMaterials = new Material[submeshObjects.Length];

        for (int i = 0; i < submeshObjects.Length; i++)
        {
            MeshFilter submeshFilter = submeshObjects[i].GetComponent<MeshFilter>();

            // 将submesh的顶点数据加到合并的数组里
            combine[i].mesh = submeshFilter.sharedMesh;
            combine[i].transform = submeshObjects[i].transform.localToWorldMatrix;

            // 设置每个submesh的材质
            combinedMaterials[i] = submeshObjects[i].GetComponent<MeshRenderer>().sharedMaterial;

            // 更新顶点偏移量
            vertexOffset += combine[i].mesh.vertexCount;
        }

        // 将所有submesh合并到一个Mesh中
        Mesh combinedMesh = new Mesh();
        combinedMesh.CombineMeshes(combine);

        // 赋给合并后的GameObject
        combinedMeshFilter.mesh = combinedMesh;

        // 设置合并后的材质（多个材质）
        combinedMeshRenderer.materials = combinedMaterials;

        // 删除所有子对象
        foreach (var submeshObj in submeshObjects)
        {
            Destroy(submeshObj);  // 销毁所有submesh对象
        }

        Debug.Log("Submeshes have been combined into one mesh.");
    }
}
