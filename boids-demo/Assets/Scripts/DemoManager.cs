using System;
using System.Collections;
using Unity.Collections;
using Unity.Mathematics;
using UnityEngine;
using Random = UnityEngine.Random;

namespace Demo.Boids
{
    // accessing CPU cache (L1 and L2) is fast
    // accessing RAM memory is slow
    // cache misses are expensive and should be avoided
    // cache misses happen when the CPU must access memory from RAM
    // to avoid cache misses flatten your data (arrays)
    // to avoid cache misses use immutable data when possible

    public class DemoManager : MonoBehaviour
    {
        [SerializeField] private Transform m_target;
        [SerializeField] private float m_targetRadius;
        private bool m_isNearTarget;
        
        // m_count is located on the heap as a value type (4 bytes)
        // since m_count is located on the heap, it requires the CPU to fetch from the RAM
        // the CPU fetches a cache line from RAM, which makes flat data (such as arrays) better for performance
        [SerializeField, ReadOnly] private int m_count;
        
        // m_prefab is located on the heap as a reference type
        // since m_count is located on the heap, it requires the CPU to fetch from the RAM (cache miss)
        // furthermore, the RAM does not store a value that can be fetched, but a address of the actual data
        [SerializeField] private GameObject m_prefab;
    
        // m_bodies is a standard reference type array that contains a pointer to data
        private Rigidbody[] m_bodies;

        [SerializeField] private float m_worldRadius;

        [SerializeField] private float m_maxSpeed;
    
        // m_positions is a value type array that contains a pointer to data
        // a native array allows programmers to do
        // 1) safety multithreading (ensuring multiple threads don't write to the same index)
        // 2) burst compilation
        private NativeArray<float3> m_velocity;
        private NativeArray<float3> m_positions;
        private NativeArray<float3> m_steering;
    
        private void Awake()
        {
            // allocator defines how long that memory is valid
            m_bodies = new Rigidbody[m_count];
            m_velocity = new NativeArray<float3>(m_count, Allocator.Persistent);
            m_positions = new NativeArray<float3>(m_count, Allocator.Persistent);
            m_steering = new NativeArray<float3>(m_count, Allocator.Persistent);
        }
    
        private void Start()
        {
            for (int i = 0; i < m_count; i++)
            {
                Vector3 randomPosition = Random.insideUnitSphere * Random.Range(0.0f, m_worldRadius);
                GameObject boid = Instantiate(m_prefab, randomPosition, Random.rotation);
                Rigidbody body = boid.GetComponent<Rigidbody>();
                m_bodies[i] = body;
                m_positions[i] = body.position;
                m_velocity[i] = Random.Range(1.0f, m_maxSpeed);
            }
        }

        private void Update()
        {
            if (!m_isNearTarget) return;
            m_target.position = Random.insideUnitSphere * Random.Range(0.0f, m_worldRadius);
            m_isNearTarget = false;
        }

        private void FixedUpdate()
        {
            for (int i = 0; i < m_count; i++)
            {
                Rigidbody body = m_bodies[i];

                m_positions[i] = body.position;
                m_velocity[i] = body.linearVelocity;
                float3 targetPosition = m_target.position;

                if (math.lengthsq(m_positions[i] - targetPosition) < m_targetRadius * m_targetRadius)
                {
                    m_isNearTarget = true;
                }

                float3 desiredVector = targetPosition - m_positions[i];
                float3 desiredVelocity = math.normalize(desiredVector) * m_maxSpeed;
                m_steering[i] = desiredVelocity - m_velocity[i];

                body.AddForce(m_steering[i], ForceMode.Force);
            }
        }

        private void OnDestroy()
        {
            m_velocity.Dispose();
            m_positions.Dispose();
            m_steering.Dispose();
        }

        private void OnDrawGizmos()
        {
            Gizmos.color = Color.green;
            Gizmos.DrawWireSphere(Vector3.zero, m_worldRadius);

            Gizmos.color = Color.red;
            Gizmos.DrawSphere(m_target.position, m_targetRadius * 0.2f);
            Gizmos.DrawWireSphere(m_target.position, m_targetRadius);
        }
    }
}