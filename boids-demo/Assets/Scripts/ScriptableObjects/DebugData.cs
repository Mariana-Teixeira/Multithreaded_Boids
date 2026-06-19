using System;
using Unity.Collections;
using Unity.Mathematics;
using UnityEngine;
using UnityEngine.Jobs;

[CreateAssetMenu(fileName = "DebugData", menuName = "Data/Debug")]
public class DebugData : ScriptableObject
{
    public bool DrawGizmos;
    public float PointRadius;

    private float3 m_position;
    private float3 m_velocity;
    private float3 m_steering;
    private float3[] m_probes;

    private void Awake()
    {
        m_probes = new float3[5];
    }

    public float3 GetPosition => m_position;
    public float3 GetVelocity => m_velocity;
    public float3 GetSteering => m_steering;
    public float3[] GetProbes => m_probes;
    
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
}
