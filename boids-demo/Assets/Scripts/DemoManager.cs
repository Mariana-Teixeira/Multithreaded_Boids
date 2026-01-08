using System;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.Serialization;
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
        // m_prefab is located on the heap as a reference type
        // since m_count is located on the heap, it requires the CPU to fetch from the RAM (cache miss)
        // furthermore, the RAM does not store a value that can be fetched, but a address of the actual data
        [Space]
        [SerializeField]
        private GameObject m_prefab;
        // m_count is located on the heap as a value type (4 bytes)
        // since m_count is located on the heap, it requires the CPU to fetch from the RAM
        // the CPU fetches a cache line from RAM, which makes flat data (such as arrays) better for performance
        [SerializeField]
        private int m_count;
        [SerializeField]
        private float m_worldRadius;
        [SerializeField]
        private float m_visionAngle;
        private float m_visionThreshold;
        [SerializeField]
        private float m_maxSpeed;
        
        private Transform[] m_transforms;
        private Vector3[] m_velocity;
        private Vector3[] m_steering;
        
        [Space]
        [SerializeField]
        private float m_separationRadius;
        [SerializeField]
        private float m_separationWeight;
        
        [Space]
        [SerializeField]
        private float m_cohesionRadius;
        [SerializeField]
        private float m_cohesionWeight;
        
        [Space]
        [SerializeField]
        private float m_alignmentRadius;
        [SerializeField]
        private float m_alignmentWeight;
        
        [Space]
        [SerializeField]
        private float m_timePrediction;
    
        private void Awake()
        {
            m_transforms = new Transform[m_count];
            m_velocity = new Vector3[m_count];
            m_steering = new Vector3[m_count];

            m_visionThreshold = Mathf.Cos(m_visionAngle * 0.5f * Mathf.Deg2Rad);
            
            #if UNITY_EDITOR
            m_debugData.m_cohesionList = new List<int>();
            m_debugData.m_alignmentList = new List<int>();
            #endif
        }
    
        private void Start()
        {
            for (int i = 0; i < m_count; i++)
            {
                Vector3 randomPosition = Random.insideUnitSphere * Random.Range(0.0f, m_worldRadius);
                GameObject boid = Instantiate(m_prefab, randomPosition, Random.rotation);
                boid.name = $"Boids_{i}";
                
                m_transforms[i] = boid.transform;
                m_velocity[i] = Random.insideUnitSphere * Random.Range(1.0f, m_maxSpeed);
            }
        }

        private void Update()
        {
            BoidUpdate();
        }

        // Craig Reinolds' Steering Behaviors For Autonomous Characters
        private void BoidUpdate()
        {            
            for (int i = 0; i < m_count; i++)
            {
                Vector3 separationForce = Vector3.zero;
                Vector3 cohesionForce = Vector3.zero;
                Vector3 alignmentForce = Vector3.zero;
                int cohesionCount = 0;
                int alignmentCount = 0;
                
                m_steering[i] = Vector3.zero;

                #if UNITY_EDITOR
                m_debugData.m_alignmentList.Clear();
                m_debugData.m_cohesionList.Clear();
                #endif
                
                for (int j = 0; j < m_count; j++)
                {
                    if (i == j) continue;
                    
                    Vector3 distanceVector = m_transforms[i].position - m_transforms[j].position;
                    float distanceSquare = Vector3.SqrMagnitude(distanceVector);

                    Vector3 reversedDistanceVector = m_transforms[j].position - m_transforms[i].position;
                    float dot = Vector3.Dot(m_velocity[i].normalized, reversedDistanceVector.normalized);
                    bool visible = dot > m_visionThreshold;
                    
                    if (visible && distanceSquare < m_separationRadius * m_separationRadius)
                    {
                        separationForce += distanceVector / distanceSquare;
                        m_steering[i] += separationForce.normalized * m_separationWeight;
                    }

                    if (visible && distanceSquare < m_cohesionRadius * m_cohesionRadius)
                    {
                        cohesionForce += m_transforms[j].position;
                        cohesionCount++;
                        
                        #if UNITY_EDITOR
                        m_debugData.m_cohesionList.Add(j);
                        #endif
                    }

                    if (visible && distanceSquare < m_alignmentRadius * m_alignmentRadius)
                    {
                        alignmentForce += m_velocity[j];
                        alignmentCount++;
                        
                        #if UNITY_EDITOR
                        m_debugData.m_alignmentList.Add(j);
                        #endif
                    }
                }

                if (cohesionCount > 0)
                {
                    cohesionForce = cohesionForce/cohesionCount - m_transforms[i].position;
                    m_steering[i] += cohesionForce.normalized * m_cohesionWeight;
                }

                if (alignmentCount > 0)
                {
                    alignmentForce /= alignmentCount;
                    m_steering[i] += alignmentForce.normalized * m_alignmentWeight;
                }
                
                Vector3 futurePosition = m_transforms[i].position + m_velocity[i] * m_timePrediction;
                if (Vector3.SqrMagnitude(futurePosition) > m_worldRadius * m_worldRadius)
                {
                    // Variable names from Kyle Halladay's "Ray-Sphere Intersection with Simple Math".
                    // World Center is at Vector3.Zero.

                    Vector3 forwardVector = m_velocity[i].normalized;
                    
                    float tc = Vector3.Dot(futurePosition, forwardVector);
                    float dSquare = futurePosition.sqrMagnitude - tc * tc;
                    float t1c = Mathf.Sqrt(m_worldRadius * m_worldRadius - dSquare);
                    
                    float t1 = tc - t1c;
                    Vector3 collisionPoint = futurePosition - forwardVector * t1;

                    Vector3 collisionNormal = (-collisionPoint).normalized;
                    Vector3 dotVector = forwardVector * Vector3.Dot(collisionNormal, forwardVector);
                    Vector3 parallel = collisionNormal - dotVector;
                    
                    m_steering[i] = parallel.normalized * m_maxSpeed;
                    
                    #if UNITY_EDITOR
                    if (m_debugData.BoidIndex == i)
                    {
                        m_debugData.Parallel = parallel;
                        m_debugData.CollisionPoint = collisionPoint;
                    }
                    #endif
                }
                
                m_velocity[i] += m_steering[i] * Time.deltaTime;
                m_velocity[i] = Vector3.ClampMagnitude(m_velocity[i], m_maxSpeed);
                m_transforms[i].position += m_velocity[i] * Time.deltaTime;
                
                #if UNITY_EDITOR
                if (m_debugData.BoidIndex == i)
                {
                    m_debugData.Position = m_transforms[i].position;
                    m_debugData.Velocity = m_velocity[i];
                    m_debugData.Steering = m_steering[i];
                    m_debugData.SeparationForce = separationForce;
                    m_debugData.CohesionForce = cohesionForce;
                    m_debugData.AlignmentForce = alignmentForce;
                    m_debugData.FuturePosition = futurePosition;
                }
                #endif
            }
        }

        #region Development Methods
        #if UNITY_EDITOR
        [Serializable]
        public struct DebugData
        {
            public bool DrawGizmos;
            public float PointRadius;
            public float ColorTransparency;
            
            [Space]
            public int BoidIndex;
            
            [HideInInspector]
            public Vector3 Position;
            [HideInInspector]
            public Vector3 Velocity;
            [HideInInspector]
            public Vector3 Steering;
            [HideInInspector]
            public Vector3 SeparationForce;
            [HideInInspector]
            public Vector3 CohesionForce;
            [HideInInspector]
            public Vector3 AlignmentForce;
            [HideInInspector]
            public Vector3 FuturePosition;
            [HideInInspector]
            public Vector3 Parallel;
            
            [Space]
            public Vector3 CollisionPoint;

            [Space]
            public List<int> m_cohesionList;
            [Space]
            public List<int> m_alignmentList;

        }

        [Space]
        [SerializeField]
        private DebugData m_debugData;

        private void OnDrawGizmos()
        {
            if (!m_debugData.DrawGizmos) return;

            Gizmos.color = new Color(1.0f, 1.0f, 1.0f, m_debugData.ColorTransparency);
            Gizmos.DrawSphere(Vector3.zero, m_worldRadius);
            
            Gizmos.color = Color.red;
            Gizmos.DrawSphere(m_debugData.Position + m_debugData.Velocity + m_debugData.Steering, m_debugData.PointRadius);
            Gizmos.DrawLine(m_debugData.Position + m_debugData.Velocity, m_debugData.Position + m_debugData.Velocity + m_debugData.Steering);
            
            Gizmos.color = Color.blue;
            Gizmos.DrawSphere(m_debugData.Position + m_debugData.Velocity, m_debugData.PointRadius);
            Gizmos.DrawLine(m_debugData.Position, m_debugData.Position + m_debugData.Velocity);
            
            Gizmos.color = Color.cyan;
            Gizmos.DrawWireSphere(m_debugData.Position, m_separationRadius);
            Gizmos.DrawLine(m_debugData.Position, m_debugData.Position + m_debugData.SeparationForce);
            Gizmos.DrawSphere(m_debugData.Position + m_debugData.SeparationForce, m_debugData.PointRadius);
            
            Gizmos.color = Color.magenta;
            Gizmos.DrawWireSphere(m_debugData.Position, m_cohesionRadius);
            Gizmos.DrawLine(m_debugData.Position, m_debugData.Position + m_debugData.CohesionForce);
            Gizmos.DrawSphere(m_debugData.Position + m_debugData.CohesionForce, m_debugData.PointRadius);

            foreach (int index in m_debugData.m_cohesionList)
            {
                Gizmos.DrawSphere(m_transforms[index].position, m_debugData.PointRadius);
            }
            
            Gizmos.color = Color.yellow;
            Gizmos.DrawWireSphere(m_debugData.Position, m_alignmentRadius);
            Gizmos.DrawLine(m_debugData.Position, m_debugData.Position + m_debugData.AlignmentForce);
            Gizmos.DrawSphere(m_debugData.Position + m_debugData.AlignmentForce, m_debugData.PointRadius);

            foreach (int index in m_debugData.m_alignmentList)
            {
                Gizmos.DrawSphere(m_transforms[index].position, m_debugData.PointRadius);
            }
            
            Gizmos.color = Color.white;
            Gizmos.DrawLine(m_debugData.Position, m_debugData.FuturePosition);
            Gizmos.DrawRay(m_debugData.Position, m_debugData.Parallel);
            Gizmos.DrawSphere(m_debugData.CollisionPoint, m_debugData.PointRadius);
        }
        #endif
        #endregion
    }
}