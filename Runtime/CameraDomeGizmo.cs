using System.Collections.Generic;
using UnityEditor;
using UnityEngine;

[ExecuteInEditMode]
public class CameraDomeGizmo : MonoBehaviour
{
    public Transform target;
    public int numRings = 4;
    public int viewsPerRing = 20;
    public float radius = 5f;
    public float height = 1.5f;
    public float gizmoSize = 0.1f;
    public int mode = 0;
    public Vector3 volumeCenter = Vector3.zero;
    public Vector3 volumeSize = new Vector3(5, 5, 5);
    public int subdivX = 2, subdivY = 2, subdivZ = 2;
    public bool showGrid = true;
    public float currentTime = 0f;
    private void OnDrawGizmos()
    {
        Gizmos.color = Color.cyan;

        if (mode == 0)
        {
            if (target == null) return;

            for (int ring = 0; ring < numRings; ring++)
            {
                float elevation = Mathf.Lerp(-Mathf.PI / 4, Mathf.PI / 4, (float)ring / (numRings - 1));

                for (int i = 0; i < viewsPerRing; i++)
                {
                    float azimuth = i * Mathf.PI * 2 / viewsPerRing;

                    float x = radius * Mathf.Cos(elevation) * Mathf.Cos(azimuth);
                    float y = radius * Mathf.Sin(elevation);
                    float z = radius * Mathf.Cos(elevation) * Mathf.Sin(azimuth);

                    Vector3 position = target.position + new Vector3(x, y + height, z);
                    Vector3 direction = (target.position - position).normalized;

                    Gizmos.DrawSphere(position, gizmoSize);
                    Gizmos.DrawLine(position, position + direction * 0.5f);
                }
            }
        }
        else
        {
            Vector3 step = new Vector3(volumeSize.x / subdivX, volumeSize.y / subdivY, volumeSize.z / subdivZ);

            List<Vector3> directions = GenerateCustomSphericalDirections(); 
          

            Gizmos.color = Color.blue;
            Gizmos.DrawWireCube(volumeCenter, volumeSize);
            if (showGrid)
                DrawSubdivisionGrid();

            Vector3 center = volumeCenter;


            Gizmos.color = Color.cyan;
            foreach (Vector3 dir in directions)
            {
                Gizmos.DrawSphere(center, gizmoSize);
                Gizmos.DrawLine(center, center + dir.normalized * 0.2f);
            }
        


    }
}

    private List<Vector3> GenerateCustomSphericalDirections()
    {
        List<Vector3> directions = new List<Vector3>();

        for (int i = 0; i < 8; i++)
        {
            float azimuth = i * 45f;
            Quaternion rot = Quaternion.Euler(0f, azimuth, 0f);
            directions.Add(rot * Vector3.forward);
        }

        for (int i = 0; i < 4; i++)
        {
            float azimuth = i * 90f;
            Quaternion rot = Quaternion.Euler(45f, azimuth, 0f);
            directions.Add(rot * Vector3.forward);
        }

        for (int i = 0; i < 4; i++)
        {
            float azimuth = i * 90f;
            Quaternion rot = Quaternion.Euler(-45f, azimuth, 0f);
            directions.Add(rot * Vector3.forward);
        }

        

        return directions;
    }

    private void DrawSubdivisionGrid()
    {
        Gizmos.color = Color.gray;

        Vector3 start = volumeCenter - volumeSize / 2f;
        Vector3 step = new Vector3(volumeSize.x / subdivX, volumeSize.y / subdivY, volumeSize.z / subdivZ);

        for (int y = 0; y <= subdivY; y++)
        {
            for (int z = 0; z <= subdivZ; z++)
            {
                Vector3 p1 = start + new Vector3(0, y * step.y, z * step.z);
                Vector3 p2 = p1 + new Vector3(volumeSize.x, 0, 0);
                Gizmos.DrawLine(p1, p2);
            }
        }

        for (int x = 0; x <= subdivX; x++)
        {
            for (int z = 0; z <= subdivZ; z++)
            {
                Vector3 p1 = start + new Vector3(x * step.x, 0, z * step.z);
                Vector3 p2 = p1 + new Vector3(0, volumeSize.y, 0);
                Gizmos.DrawLine(p1, p2);
            }
        }

        for (int x = 0; x <= subdivX; x++)
        {
            for (int y = 0; y <= subdivY; y++)
            {
                Vector3 p1 = start + new Vector3(x * step.x, y * step.y, 0);
                Vector3 p2 = p1 + new Vector3(0, 0, volumeSize.z);
                Gizmos.DrawLine(p1, p2);
            }
        }
    }

    private void Update()
    {
        #if UNITY_EDITOR
        if (EditorApplication.isPlaying && !EditorApplication.isPaused)
            currentTime += Time.deltaTime;
        else
            currentTime = 0;
        #endif
    }
}
