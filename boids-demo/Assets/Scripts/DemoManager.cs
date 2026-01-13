using System;
using System.Linq;
using UnityEngine;
using Random = UnityEngine.Random;

namespace Demo.Boids
{
    public class DemoManager : MonoBehaviour
    {
        [Serializable]
        private struct WorldData
        {
            public float WorldRadius;
            public float SpawnRadius;
            public int Count;
        }
        
        [Serializable]
        private struct BoidData
        {
            public GameObject Prefab;
            
            [Header("Move")]
            public float MinSpeed;
            public float MaxSpeed;

            [Header("Spatial Grid (WIP)")]
            public float VisionRadius;

            [Header("Steering")]
            public float SeparationRadius;
            [Range(-1, 1)]
            public float SeparationDot;
            public float SeparationWeight;

            [Space]
            public float CohesionRadius;
            [Range(-1, 1)]
            public float CohesionDot;
            public float CohesionWeight;

            [Space]
            public float AlignmentRadius;
            [Range(-1, 1)]
            public float AlignmentDot;
            public float AlignmentWeight;

            [Header("Wander")]
            public float WanderLength;
            public float WanderRadius;
            public float WanderRate;
            
            [Header("Contain")]
            public float ContainLength;
            public float ContainAngle;
        }

        [Space, SerializeField]
        private WorldData m_worldData;
        [Space, SerializeField]
        private BoidData m_boidData;
        [Space, SerializeField]
        private GameState m_state;
        
        private Transform[] m_transforms;
        private Vector3[] m_velocities;
        private Vector3[] m_steerings;
            
        private Vector3[] m_containmentProbes;
        private const int PROBE_OFFSET = 5;
    
        private void Awake()
        {
            m_transforms = new Transform[m_worldData.Count];
            m_velocities = new Vector3[m_worldData.Count];
            m_steerings = new Vector3[m_worldData.Count];

            m_containmentProbes = new Vector3[m_worldData.Count * 5];
        }
    
        private void Start()
        {
            for (int i = 0; i < m_worldData.Count; i++)
            {
                Vector3 randomPosition = Random.insideUnitSphere * m_worldData.SpawnRadius;
                GameObject boid = Instantiate(m_boidData.Prefab, randomPosition, Quaternion.identity);
                boid.name = $"Boids_{i}";
                
                m_transforms[i] = boid.transform;
            }
        }

        private void Update()
        {
#if UNITY_EDITOR
            m_debugData.CleanData();
#endif

            int[] neighbours = new int[m_worldData.Count];

            for (int i = 0; i < m_worldData.Count; i++)
            {
                UpdateGrid(i, neighbours, out int count);
                UpdateProbes(i);
                UpdateLogic(i, neighbours, count);
                UpdateMovement(i);
                UpdateRotation(i);
            }
        }

        private void UpdateGrid(int index, int[] neighbours, out int count)
        {
            count = 0;
            for (int j = 0; j < m_worldData.Count; j++)
            {
                if (index == j) continue;
                
                Vector3 vectorToNeighbour = m_transforms[j].position - m_transforms[index].position;
                bool insideVisionRadius = vectorToNeighbour.sqrMagnitude < m_boidData.VisionRadius * m_boidData.VisionRadius;
                
                if (insideVisionRadius)
                {
                    neighbours[count] = j;
                    count++;
                }
            }
        }

        private void UpdateProbes(int index)
        {
            Vector3 originPosition = m_transforms[index].position;
            Vector3 forward = m_velocities[index].normalized;
            float length = m_velocities[index].magnitude * m_boidData.ContainLength;
            float spreadAngle = m_boidData.ContainAngle;
            Quaternion velocityRotation = Quaternion.LookRotation(forward, m_transforms[index].up);

            m_containmentProbes[PROBE_OFFSET * index + 0] = originPosition + forward * length;
            m_containmentProbes[PROBE_OFFSET * index + 1] = originPosition + velocityRotation * Quaternion.AngleAxis(-spreadAngle, Vector3.right) * Vector3.forward * length;
            m_containmentProbes[PROBE_OFFSET * index + 2] = originPosition + velocityRotation * Quaternion.AngleAxis(spreadAngle, Vector3.right) * Vector3.forward * length;
            m_containmentProbes[PROBE_OFFSET * index + 3] = originPosition + velocityRotation * Quaternion.AngleAxis(-spreadAngle, Vector3.up) * Vector3.forward * length;
            m_containmentProbes[PROBE_OFFSET * index + 4] = originPosition + velocityRotation * Quaternion.AngleAxis(spreadAngle, Vector3.up) * Vector3.forward * length;
            
#if UNITY_EDITOR
            if (m_debugData.BoidIndex == index)
            {
                m_debugData.ContainProbes = new[]
                {
                    m_containmentProbes[PROBE_OFFSET * index + 0],
                    m_containmentProbes[PROBE_OFFSET * index + 1],
                    m_containmentProbes[PROBE_OFFSET * index + 2],
                    m_containmentProbes[PROBE_OFFSET * index + 3],
                    m_containmentProbes[PROBE_OFFSET * index + 4]
                };
            }
#endif
        }

        private void UpdateLogic(int index, int[] neighbours, int neighbourCount)
        {    
            m_steerings[index] = GetFlockSteering(index, m_transforms, m_velocities, neighbours, neighbourCount, m_boidData);
            m_steerings[index] += GetContainment();
            
            Vector3 GetContainment()
            {
                
                Vector3[] myProbes =
                {
                    m_containmentProbes[PROBE_OFFSET * index + 0],
                    m_containmentProbes[PROBE_OFFSET * index + 1],
                    m_containmentProbes[PROBE_OFFSET * index + 2],
                    m_containmentProbes[PROBE_OFFSET * index + 3],
                    m_containmentProbes[PROBE_OFFSET * index + 4]
                };

                Vector3 activeProbe = myProbes.FirstOrDefault(x => x.sqrMagnitude > m_worldData.WorldRadius * m_worldData.WorldRadius);
                if (activeProbe != default)
                     return GetContainmentSteering(index, m_transforms[index].position, m_velocities[index], activeProbe, m_worldData);
                return Vector3.zero;
            }
        }

        private void UpdateMovement(int index)
        {
            m_velocities[index] += m_steerings[index] * Time.deltaTime;
            m_velocities[index] = Vector3.ClampMagnitude(m_velocities[index], m_boidData.MaxSpeed);
            m_transforms[index].position += m_velocities[index] * Time.deltaTime;
                
#if UNITY_EDITOR
            if (m_debugData.BoidIndex == index)
            {
                m_debugData.Position = m_transforms[index].position;
                m_debugData.Velocity = m_velocities[index];
                m_debugData.Steering = m_steerings[index];
            }
#endif
        }
        
        private void UpdateRotation(int index)
        {
            Quaternion lookAt = Quaternion.LookRotation(m_velocities[index]);
            m_transforms[index].rotation = lookAt;
        }
        
        private Vector3 GetFlockSteering(
            int index, ReadOnlySpan<Transform> transforms, ReadOnlySpan<Vector3> velocities,
            int[] neighbours, int neighbourCount,
            BoidData boidData)
        {
            Vector3 steering = new Vector3();
            Vector3 separationForce = new Vector3();
            Vector3 cohesionForce = new Vector3();
            Vector3 alignmentForce = new Vector3();

            int cohesionCount = 0;
            int alignmentCount = 0;
            
            for (int j = 0; j < neighbourCount; j++)
            {
                int nIndex = neighbours[j];
                    
                Vector3 vectorToNeighbour = transforms[nIndex].position - transforms[index].position;
                Vector3 vectorFromNeighbour = vectorToNeighbour * -1.0f;
    
                float dot = Vector3.Dot(velocities[index].normalized, vectorToNeighbour.normalized);
                if (vectorToNeighbour.sqrMagnitude < m_boidData.SeparationRadius * m_boidData.SeparationRadius &&
                    dot > m_boidData.SeparationDot)
                {
                    separationForce += vectorFromNeighbour / vectorFromNeighbour.magnitude;
                }

                if (vectorToNeighbour.sqrMagnitude < m_boidData.CohesionRadius * m_boidData.CohesionRadius &&
                    dot > m_boidData.CohesionDot)
                {
                    cohesionForce += transforms[nIndex].position;
                    cohesionCount++;   
                }

                if (vectorToNeighbour.sqrMagnitude < m_boidData.AlignmentRadius * m_boidData.AlignmentRadius &&
                    dot > m_boidData.AlignmentDot)
                {
                    alignmentForce += velocities[nIndex];
                    alignmentCount++;
                }
            }
            
            steering += separationForce * boidData.SeparationWeight;

            if (cohesionCount > 0)
            {
                cohesionForce = cohesionForce/cohesionCount - transforms[index].position;
                steering += cohesionForce * boidData.CohesionWeight;
            }

            if (alignmentCount > 0)
            {
                alignmentForce /= alignmentCount;
                steering += alignmentForce * boidData.AlignmentWeight;
            }
            
#if UNITY_EDITOR
            if (m_debugData.BoidIndex == index)
            {
                m_debugData.SeparationForce = separationForce;
                m_debugData.CohesionForce = cohesionForce;
                m_debugData.AlignmentForce = alignmentForce;
            }
#endif
            return steering.normalized * m_boidData.MaxSpeed;
        }

        private Vector3 GetContainmentSteering(
            int index, Vector3 position, Vector3 velocity,
            Vector3 activeProbe, WorldData worldData)
        {
            Vector3 collisionPoint = GetCollisionPoint(position, (activeProbe - position).normalized, worldData.WorldRadius);
            Vector3 perpendicular = GetPerpendicular(-collisionPoint.normalized, -velocity.normalized);
            
#if UNITY_EDITOR
            if (m_debugData.BoidIndex == index)
            {
                m_debugData.ContainPerpendicular = perpendicular;
                m_debugData.ContainCollisionPoint = collisionPoint;
            }
#endif
            
            Vector3 GetCollisionPoint(Vector3 origin, Vector3 rayDirection, float radius)
            {
                float a = 1;
                float b = 2.0f * Vector3.Dot(rayDirection, origin);
                float c = Vector3.Dot(origin, origin) - radius * radius;
                SolveQuadratic(a, b, c, out float t0, out float t1);

                return origin + rayDirection * t1;
            }
            
            void SolveQuadratic(float a, float b, float c, out float t0, out float t1)
            {
                t0 = t1 = 0;
                
                float discriminant = b * b - 4 * a * c;

                if (discriminant < 0) return;
                
                if (discriminant == 0) 
                {
                    t0 = t1 = -0.5f * b / a;
                }
                else 
                {
                    float q = b > 0 ? 
                        -0.5f * (b + Mathf.Sqrt(discriminant)) : 
                        -0.5f * (b - Mathf.Sqrt(discriminant));
                    t0 = q / a;
                    t1 = c / q;
                }

                if (t0 > t1) (t0, t1) = (t1, t0);
            }

            Vector3 GetPerpendicular(Vector3 collisionNormal, Vector3 normalForward)
            {
                Vector3 dotVector = normalForward * Vector3.Dot(collisionNormal, normalForward);
                return (collisionNormal - dotVector).normalized;
            }

            return perpendicular * m_boidData.MaxSpeed;
        }

#region Development Methods
#if UNITY_EDITOR
        [Serializable]
        public struct DebugData
        {
            public bool DrawGizmos;
            public bool DrawBoidGizmos;
            public float PointRadius;
            
            [Space]
            public int BoidIndex;

            [NonSerialized]
            public Vector3 Position;
            [NonSerialized]
            public Vector3 Velocity;
            [NonSerialized]
            public Vector3 Steering;
            [NonSerialized]
            public Vector3 SeparationForce;
            [NonSerialized]
            public Vector3 CohesionForce;
            [NonSerialized]
            public Vector3 AlignmentForce;
            [NonSerialized]
            public Vector3[] ContainProbes;
            [NonSerialized]
            public Vector3 ContainPerpendicular;
            [NonSerialized]
            public Vector3 ContainCollisionPoint;

            public void CleanData()
            {
                ContainPerpendicular = Position;
                ContainCollisionPoint = Position;
                
                SeparationForce = Vector3.zero;
                CohesionForce = Vector3.zero;
                AlignmentForce = Vector3.zero;
                
                Position = Vector3.zero;
                Velocity = Vector3.zero;
                Steering = Vector3.zero;
            }
        }

        [Space]
        [SerializeField]
        private DebugData m_debugData;

        private void OnDrawGizmos()
        {
            if (!m_debugData.DrawGizmos) return;

            Gizmos.color = Color.black;
            Gizmos.DrawWireSphere(Vector3.zero, m_worldData.WorldRadius);
            Gizmos.DrawWireSphere(Vector3.zero, m_worldData.SpawnRadius);

            if (!m_debugData.DrawBoidGizmos) return;
            
            Gizmos.color = Color.red;
            Gizmos.DrawSphere(m_debugData.Position + m_debugData.Velocity + m_debugData.Steering, m_debugData.PointRadius);
            Gizmos.DrawLine(m_debugData.Position + m_debugData.Velocity, m_debugData.Position + m_debugData.Velocity + m_debugData.Steering);
            
            Gizmos.color = Color.blue;
            Gizmos.DrawSphere(m_debugData.Position + m_debugData.Velocity, m_debugData.PointRadius);
            Gizmos.DrawLine(m_debugData.Position, m_debugData.Position + m_debugData.Velocity);
            
            Gizmos.color = Color.green;
            if (m_debugData.ContainProbes != null)
            {
                for (int i = 0; i < m_debugData.ContainProbes.Length; i++)
                {
                    Gizmos.DrawLine(m_debugData.Position, m_debugData.ContainProbes[i]);
                }
            }

            DrawContain();
            DrawFlock();
        }
        
        private void DrawContain()
        {
            Gizmos.color = Color.white;
            Gizmos.DrawRay(m_debugData.Position, m_debugData.ContainPerpendicular);
            Gizmos.DrawLine(m_debugData.Position, m_debugData.ContainCollisionPoint);
            Gizmos.DrawSphere(m_debugData.ContainCollisionPoint, m_debugData.PointRadius);
        }

        private void DrawFlock()
        {
            Gizmos.color = Color.cyan;
            Gizmos.DrawLine(m_debugData.Position, m_debugData.Position + m_debugData.SeparationForce);
            Gizmos.DrawSphere(m_debugData.Position + m_debugData.SeparationForce, m_debugData.PointRadius);
            
            Gizmos.color = Color.magenta;
            Gizmos.DrawLine(m_debugData.Position, m_debugData.Position + m_debugData.CohesionForce);
            Gizmos.DrawSphere(m_debugData.Position + m_debugData.CohesionForce, m_debugData.PointRadius);
            
            Gizmos.color = Color.yellow;
            Gizmos.DrawLine(m_debugData.Position, m_debugData.Position + m_debugData.AlignmentForce);
            Gizmos.DrawSphere(m_debugData.Position + m_debugData.AlignmentForce, m_debugData.PointRadius);
        }
#endif
#endregion
    }
}