using System;
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
        [SerializeField]
        private Transform m_target;
        [SerializeField, Range(0, 1)]
        private float m_targetSpeed;
        private float m_targetAngle;
        
        [SerializeField]
        private float m_visionRadius;
        private float m_visionRadiusSquare;
        
        // m_count is located on the heap as a value type (4 bytes)
        // since m_count is located on the heap, it requires the CPU to fetch from the RAM
        // the CPU fetches a cache line from RAM, which makes flat data (such as arrays) better for performance
        [SerializeField]
        private int m_count;
        
        // m_prefab is located on the heap as a reference type
        // since m_count is located on the heap, it requires the CPU to fetch from the RAM (cache miss)
        // furthermore, the RAM does not store a value that can be fetched, but a address of the actual data
        [SerializeField]
        private GameObject m_prefab;

        [SerializeField]
        private float m_worldRadius;
        [SerializeField]
        private float m_maxSpeed;
        
        private Transform[] m_transforms;
        private Vector3[] m_velocity;
        private Vector3[] m_steering;
    
        private void Awake()
        {
            m_transforms = new Transform[m_count];
            m_velocity = new Vector3[m_count];
            m_steering = new Vector3[m_count];

            m_visionRadiusSquare = m_visionRadius * m_visionRadius;
        }
    
        private void Start()
        {
            for (int i = 0; i < m_count; i++)
            {
                Vector3 randomPosition = Random.insideUnitSphere * Random.Range(0.0f, m_worldRadius);
                GameObject boid = Instantiate(m_prefab, randomPosition, Random.rotation);
                m_transforms[i] = boid.transform;
                m_velocity[i] = Random.insideUnitSphere * Random.Range(1.0f, m_maxSpeed);
            }
        }

        private void Update()
        {
            TargetUpdate();
            BoidUpdate();
            
            #if UNITY_EDITOR
            m_debugData.Position = m_transforms[m_debugData.BoidIndex].position;
            m_debugData.Velocity = m_velocity[m_debugData.BoidIndex];
            m_debugData.Steering = m_steering[m_debugData.BoidIndex];
            #endif
        }

        private void TargetUpdate()
        {
            m_targetAngle += m_targetSpeed * Time.deltaTime;
            m_target.position = new Vector3(Mathf.Cos(m_targetAngle) * m_worldRadius,
                                            Mathf.Sin(m_targetAngle) * m_worldRadius,
                                            0.0f);
        }

        private void BoidUpdate()
        {
            for (int i = 0; i < m_count; i++)
            {
                Vector3 cohesionVector = Vector3.zero;
                short count = 1;
                
                for (int j = 0; j < m_count; j++)
                {
                    if (i == j) continue;
                    if (Vector3.SqrMagnitude(m_transforms[i].position - m_transforms[j].position) > m_visionRadiusSquare) continue;

                    cohesionVector += m_transforms[j].position - m_transforms[i].position;
                    count++;
                }

                cohesionVector /= count;

                m_steering[i] = cohesionVector - m_velocity[i];
                
                m_velocity[i] += m_steering[i] * Time.deltaTime;
                m_transforms[i].position += m_velocity[i] * Time.deltaTime;
            }
        }

        #region Development Methods
        #if UNITY_EDITOR
        [Serializable]
        public struct DebugData
        {
            public bool DrawGizmos;
            public float PointRadius;
            
            public int BoidIndex;
            public float TargetRadius;

            [HideInInspector]
            public Vector3 Position;
            [HideInInspector]
            public Vector3 Velocity;
            [HideInInspector]
            public Vector3 Steering;
        }

        [SerializeField]
        private DebugData m_debugData;

        private void OnDrawGizmos()
        {
            if (!m_debugData.DrawGizmos) return;
            
            Gizmos.color = Color.black;
            Gizmos.DrawWireSphere(Vector3.zero, m_worldRadius);
            Gizmos.DrawWireSphere(m_target.position, m_debugData.TargetRadius);
            
            Gizmos.color = Color.blue;
            Gizmos.DrawWireSphere(m_debugData.Position, m_visionRadius);
            Gizmos.DrawLine(m_debugData.Position, m_debugData.Position + m_debugData.Velocity);
            Gizmos.DrawSphere(m_debugData.Position + m_debugData.Velocity, m_debugData.PointRadius);
            Gizmos.DrawLine(m_debugData.Position, m_debugData.Position + m_debugData.Steering);
        }
        #endif
        #endregion
    }
}