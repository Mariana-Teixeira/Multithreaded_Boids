using Unity.Collections;
using Unity.Mathematics;
using UnityEngine;

public class DemoDebug : MonoBehaviour
{
#if UNITY_EDITOR
    
    [SerializeField] private bool m_drawGizmos = true;
    [SerializeField] private float m_pointRadius = 5.0f;

    private float m_worldRadius;
    private float m_spawnRadius;
    private float3 m_position;
    private float3 m_velocity;
    private float3 m_steering;
    private readonly float3[] m_probes = new float3[5];

    public void SetWorldData(float worldRadius, float spawnRadius)
    {
        m_worldRadius = worldRadius;
        m_spawnRadius = spawnRadius;
    }
    
    private void OnDrawGizmos()
    {
        if (!m_drawGizmos) return;

        DrawWorld();
        DrawSteering();
        DrawVelocity();
        DrawProbes();
    }
    
    public void UpdateDebug(NativeArray<float3> positions, NativeArray<float3> velocities, NativeArray<float3> steerings, NativeArray<float3> probes)
    {
        m_position = positions[0];
        m_velocity = velocities[0];
        m_steering = steerings[0];

        for (int i = 0; i < 5; i++)
        {
            m_probes[i] = probes[i];
        }
    }

    private void DrawWorld()
    {
        Gizmos.color = Color.black;
        Gizmos.DrawWireSphere(float3.zero, m_worldRadius);
        Gizmos.DrawWireSphere(float3.zero, m_spawnRadius);
    }

    private void DrawSteering()
    {
        Gizmos.color = Color.red;
        Gizmos.DrawSphere(m_position + m_velocity + m_steering, m_pointRadius);
        Gizmos.DrawLine(m_position + m_velocity, m_position + m_velocity + m_steering);
    }

    private void DrawVelocity()
    {
        Gizmos.color = Color.blue;
        Gizmos.DrawSphere(m_position + m_velocity, m_pointRadius);
        Gizmos.DrawLine(m_position, m_position + m_velocity);
    }

    private void DrawProbes()
    {
        Gizmos.color = Color.green;
        Gizmos.DrawLine(m_position, m_probes[0]);
        Gizmos.DrawLine(m_position, m_probes[1]);
        Gizmos.DrawLine(m_position, m_probes[2]);
        Gizmos.DrawLine(m_position, m_probes[3]);
        Gizmos.DrawLine(m_position, m_probes[4]);
    }
#endif
}
