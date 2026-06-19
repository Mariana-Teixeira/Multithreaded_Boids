using Unity.Mathematics;
using UnityEngine;

public class DemoDebug : MonoBehaviour
{
#if UNITY_EDITOR
    [SerializeField] private WorldData m_worldData;
    [SerializeField] private DebugData m_debugData;

    private void OnDrawGizmos()
    {
        if (!m_debugData.DrawGizmos) return;

        DrawWorld();
        DrawSteering();
        DrawVelocity();
        DrawProbes();
    }

    private void DrawWorld()
    {
        Gizmos.color = Color.black;
        Gizmos.DrawWireSphere(float3.zero, m_worldData.WorldRadius);
        Gizmos.DrawWireSphere(float3.zero, m_worldData.SpawnRadius);
    }

    private void DrawSteering()
    {
        Gizmos.color = Color.red;
        Gizmos.DrawSphere(m_debugData.GetPosition + m_debugData.GetVelocity + m_debugData.GetSteering, m_debugData.PointRadius);
        Gizmos.DrawLine(m_debugData.GetPosition + m_debugData.GetVelocity, m_debugData.GetPosition + m_debugData.GetVelocity + m_debugData.GetSteering);
    }

    private void DrawVelocity()
    {
        Gizmos.color = Color.blue;
        Gizmos.DrawSphere(m_debugData.GetPosition + m_debugData.GetVelocity, m_debugData.PointRadius);
        Gizmos.DrawLine(m_debugData.GetPosition, m_debugData.GetPosition + m_debugData.GetVelocity);
    }

    private void DrawProbes()
    {
        Gizmos.color = Color.green;
        Gizmos.DrawLine(m_debugData.GetPosition, m_debugData.GetProbes[0]);
        Gizmos.DrawLine(m_debugData.GetPosition, m_debugData.GetProbes[1]);
        Gizmos.DrawLine(m_debugData.GetPosition, m_debugData.GetProbes[2]);
        Gizmos.DrawLine(m_debugData.GetPosition, m_debugData.GetProbes[3]);
        Gizmos.DrawLine(m_debugData.GetPosition, m_debugData.GetProbes[4]);
    }
#endif
}
