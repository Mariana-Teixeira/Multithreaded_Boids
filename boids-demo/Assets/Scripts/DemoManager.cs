using System;
using System.Collections.Generic;
using Unity.Mathematics;
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
            [Header("Move")]
            public float MinSpeed;
            public float MaxSpeed;
            
            [Header("Probes")]
            public float ProbeLengthMultiplier;
            public float ProbeAngle;

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
        }
        
        [SerializeField]
        private GameObject m_prefab;
        [Space, SerializeField]
        private WorldData m_worldData;
        [Space, SerializeField]
        private BoidData m_boidData;
        
        private Transform[] m_transforms;
        private Quaternion[] m_rotations;
        private Vector3[] m_positions;
        private Vector3[] m_velocities;
        private Vector3[] m_steerings;
        private Vector3[] m_probes;

        private Dictionary<int, List<int>> m_spatialGrid;
        public float m_cellSize;
        
        private const int PROBES_PER_BOID = 5;
    
        private void Awake()
        {
            m_transforms = new Transform[m_worldData.Count];
            m_positions = new Vector3[m_worldData.Count];
            m_rotations = new Quaternion[m_worldData.Count];
            m_velocities = new Vector3[m_worldData.Count];
            m_steerings = new Vector3[m_worldData.Count];
            m_probes = new Vector3[m_worldData.Count * PROBES_PER_BOID];

            m_spatialGrid = new Dictionary<int, List<int>>();
        }

        private void Start()
        {
            InstantiateBoids();

            m_cellSize = Mathf.Max(m_boidData.SeparationRadius, m_boidData.CohesionRadius, m_boidData.AlignmentRadius);
            
            Cursor.lockState = CursorLockMode.Locked;
        }
        
        private void InstantiateBoids()
        {
            for (int index = 0; index < m_worldData.Count; index++)
            {
                Vector3 randomPosition = Random.insideUnitSphere * m_worldData.SpawnRadius;
                Quaternion randomRotation = Random.rotationUniform;
                
                GameObject boid = Instantiate(m_prefab, randomPosition, randomRotation);
                boid.name = $"Boids_{index}";
                
                float speed = Random.Range(m_boidData.MinSpeed, m_boidData.MaxSpeed);
                Vector3 randomVelocity = boid.transform.forward * speed;

                m_transforms[index] = boid.transform;
                m_positions[index] = randomPosition;
                m_rotations[index] = randomRotation;
                m_velocities[index] = randomVelocity;
            }
        }

        private void Update()
        {
#if UNITY_EDITOR
            int index = m_debugData.BoidIndex;
            m_debugData.Position = m_transforms[index].position;
            m_debugData.Velocity = m_velocities[index];
            m_debugData.Steering = m_steerings[index];
            m_debugData.Probe0 = m_probes[PROBES_PER_BOID * index];
            m_debugData.Probe1 = m_probes[PROBES_PER_BOID * index + 1];
            m_debugData.Probe2 = m_probes[PROBES_PER_BOID * index + 2];
            m_debugData.Probe3 = m_probes[PROBES_PER_BOID * index + 3];
            m_debugData.Probe4 = m_probes[PROBES_PER_BOID * index + 4];
#endif
            
            m_spatialGrid.Clear();
            foreach (List<int> indexList in m_spatialGrid.Values)
            {
                indexList.Clear();
            }

            for (int i = 0; i < m_worldData.Count; i++)
            {
                UpdateSpatialGrid(i);
                UpdateProbes(i, m_boidData);
                UpdateFlockSteering(i, m_boidData);
                UpdateContainmentSteering(i, m_worldData, m_boidData);
                UpdateMovement(i, m_boidData);
            }
        }
        
        private void UpdateSpatialGrid(int index)
        {
            Vector3Int gridPosition = new Vector3Int(
                Mathf.FloorToInt(m_positions[index].x / m_cellSize),
                Mathf.FloorToInt(m_positions[index].y / m_cellSize),
                Mathf.FloorToInt(m_positions[index].z / m_cellSize));

            int key = GetHash(gridPosition);

            if (m_spatialGrid.ContainsKey(key) == false)
            {
                var indexList = new List<int>();
                m_spatialGrid.Add(key, indexList);
            }

            m_spatialGrid[key].Add(index);
        }
        
        private void UpdateProbes(int index, BoidData data)
        {
            Vector3 originPosition = m_positions[index];
            float length = m_velocities[index].magnitude * data.ProbeLengthMultiplier;
            Vector3 forward = m_velocities[index].normalized;
            Vector3 ray = forward * length;

            Vector3 globalUp = Vector3.up;
            Vector3 cross = Vector3.Cross(globalUp, forward);
            Vector3 right = cross.normalized;
            Vector3 up = Vector3.Cross(forward, right);
            
            m_probes[index * PROBES_PER_BOID] = originPosition + ray;
            
            Quaternion upTilt = Quaternion.AngleAxis(data.ProbeAngle, right);
            m_probes[index * PROBES_PER_BOID + 1] = originPosition + upTilt * ray;

            Quaternion downTilt = Quaternion.AngleAxis(-data.ProbeAngle, right);
            m_probes[index * PROBES_PER_BOID + 2] = originPosition + downTilt * ray;

            Quaternion rightTilt = Quaternion.AngleAxis(data.ProbeAngle, up);
            m_probes[index * PROBES_PER_BOID + 3] = originPosition + rightTilt * ray;

            Quaternion leftTilt = Quaternion.AngleAxis(-data.ProbeAngle, up);
            m_probes[index * PROBES_PER_BOID + 4] = originPosition + leftTilt * ray;
        }

        private void UpdateFlockSteering(int index, BoidData boidData)
        {
            Vector3 steeringVector = new Vector3();
            Vector3 separationForce = new Vector3();
            Vector3 cohesionForce = new Vector3();
            Vector3 alignmentForce = new Vector3();
            
            int cohesionCount = 0;
            int alignmentCount = 0;
            
            Vector3Int myGridPosition = new Vector3Int(
                Mathf.FloorToInt(m_positions[index].x / m_cellSize),
                Mathf.FloorToInt(m_positions[index].y / m_cellSize),
                Mathf.FloorToInt(m_positions[index].z / m_cellSize));
            
            for (int x = -1; x <= 1; x++)
            for (int y = -1; y <= 1; y++)
            for (int z = -1; z <= 1; z++)
            {
                Vector3Int otherGridPosition = myGridPosition + new Vector3Int(x, y, z);
                int hash = GetHash(otherGridPosition);

                bool hasHashList = m_spatialGrid.TryGetValue(hash, out var list);
                if (!hasHashList) continue;

                int count = list.Count;
                for (int i = 0; i < count; i++)
                {
                    int other = list[i];
                    if (index == other) continue;

                    Vector3 vectorToNeighbour = m_positions[other] - m_positions[index];
                    float distanceSqToNeighbour = vectorToNeighbour.sqrMagnitude;

                    float dot = Vector3.Dot(m_velocities[index].normalized, vectorToNeighbour.normalized);

                    if (distanceSqToNeighbour < boidData.SeparationRadius * boidData.SeparationRadius && dot > boidData.SeparationDot)
                    {
                        float distanceToNeighbour = Mathf.Sqrt(distanceSqToNeighbour);
                        separationForce += -vectorToNeighbour / distanceToNeighbour;
                    }

                    if (distanceSqToNeighbour < boidData.CohesionRadius * boidData.CohesionRadius && dot > boidData.CohesionDot)
                    {
                        cohesionForce += m_positions[other];
                        cohesionCount++;
                    }

                    if (distanceSqToNeighbour < boidData.AlignmentRadius * boidData.AlignmentRadius && dot > boidData.AlignmentDot)
                    {
                        alignmentForce += m_velocities[other];
                        alignmentCount++;
                    }
                }
            }

            steeringVector += separationForce * boidData.SeparationWeight;

            if (cohesionCount > 0)
            {
                cohesionForce = cohesionForce / cohesionCount - m_positions[index];
                steeringVector += cohesionForce * boidData.CohesionWeight;
            }

            if (alignmentCount > 0)
            {
                alignmentForce = alignmentForce / alignmentCount;
                steeringVector += alignmentForce * boidData.AlignmentWeight;
            }

            m_steerings[index] = steeringVector.normalized * boidData.MaxSpeed;
        }

        private void UpdateContainmentSteering(int index, WorldData worldData, BoidData boidData)
        {
            for (int i = 0; i < PROBES_PER_BOID; i++)
            {
                Vector3 probe = m_probes[index * PROBES_PER_BOID + i];
                if (probe.sqrMagnitude < worldData.WorldRadius * worldData.WorldRadius) continue;

                Vector3 probeDirection = (probe - m_positions[index]).normalized;
                Vector3 collisionPoint = GetCollisionPoint(m_positions[index], probeDirection, worldData.WorldRadius);

                Vector3 collisionNormal = -collisionPoint.normalized;
                Vector3 velocityNormal = -m_velocities[index].normalized;
                Vector3 perpendicular = GetPerpendicular(collisionNormal, velocityNormal);
            
                m_steerings[index] += perpendicular * boidData.MaxSpeed;
                return;
            }

            Vector3 GetCollisionPoint(Vector3 rayOrigin, Vector3 rayDirection, float colliderRadius)
            {
                float a = 1;
                float b = 2.0f * Vector3.Dot(rayDirection, rayOrigin);
                float c = Vector3.Dot(rayOrigin, rayOrigin) - colliderRadius * colliderRadius;
                float t = SolveQuadratic(a, b, c);

                return rayOrigin + rayDirection * t;
            }

            float SolveQuadratic(float a, float b, float c)
            {
                float discriminant = b * b - 4 * a * c;

                float q = b > 0 ? 
                    -0.5f * (b + Mathf.Sqrt(discriminant)) : 
                    -0.5f * (b - Mathf.Sqrt(discriminant));
                
                return c / q;
            } 
            
            Vector3 GetPerpendicular(Vector3 collisionNormal, Vector3 forwardNormal)
            {
                Vector3 dotVector = forwardNormal * Vector3.Dot(collisionNormal, forwardNormal);
                return (collisionNormal - dotVector).normalized;
            }
        }

        private void UpdateMovement(int index, BoidData data)
        {
            m_velocities[index] += m_steerings[index] * Time.deltaTime;

            float velocitySq = m_velocities[index].sqrMagnitude;
            Vector3 forward = m_velocities[index].normalized;
            if (velocitySq > data.MaxSpeed * data.MaxSpeed)
                m_velocities[index] = forward * data.MaxSpeed;
            
            m_positions[index] += m_velocities[index] * Time.deltaTime;
            m_transforms[index].position = m_positions[index];
            
            m_rotations[index] = Quaternion.LookRotation(forward, Vector3.up);
            m_transforms[index].rotation = m_rotations[index];
        }
        
        private static int GetHash(Vector3Int gridPosition)
        {
            return HashCode.Combine(gridPosition);
        }
        
        #region Debugging Methods
#if UNITY_EDITOR
        /// <summary>
        /// Struct containing all data necessary for the Gizmos.
        /// </summary>
        [Serializable]
        public struct DebugData
        {
            public bool DrawGizmos;
            public GizmosType GizmosType;
            
            [Space]
            public float PointRadius;

            [Space]
            public int BoidIndex;

            [NonSerialized] public Vector3 Position;
            [NonSerialized] public Vector3 Velocity;
            [NonSerialized] public Vector3 Steering;
            [NonSerialized] public Vector3 Probe0;
            [NonSerialized] public Vector3 Probe1;
            [NonSerialized] public Vector3 Probe2;
            [NonSerialized] public Vector3 Probe3;
            [NonSerialized] public Vector3 Probe4;
        }

        /// <summary>
        /// Struct with booleans to filter draw calls of the Gizmos.
        /// </summary>
        [Serializable]
        public struct GizmosType
        {
            public bool WorldGizmo;
            public bool SteeringGizmos;
            public bool VelocityGizmos;
            public bool ProbesGizmos;
        }

        [Space]
        [SerializeField]
        private DebugData m_debugData;

        private void OnDrawGizmos()
        {
            if (!m_debugData.DrawGizmos) return;

            if (m_debugData.GizmosType.WorldGizmo) DrawWorld();
            if (m_debugData.GizmosType.SteeringGizmos) DrawSteering();
            if (m_debugData.GizmosType.VelocityGizmos) DrawVelocity();
            if (m_debugData.GizmosType.ProbesGizmos) DrawProbes();
        }

        private void DrawWorld()
        {
            Gizmos.color = Color.black;
            Gizmos.DrawWireSphere(Vector3.zero, m_worldData.WorldRadius);
            Gizmos.DrawWireSphere(Vector3.zero, m_worldData.SpawnRadius);
        }

        private void DrawSteering()
        {
            Gizmos.color = Color.red;
            Gizmos.DrawSphere(m_debugData.Position + m_debugData.Velocity + m_debugData.Steering, m_debugData.PointRadius);
            Gizmos.DrawLine(m_debugData.Position + m_debugData.Velocity, m_debugData.Position + m_debugData.Velocity + m_debugData.Steering);
        }

        private void DrawVelocity()
        {
            Gizmos.color = Color.blue;
            Gizmos.DrawSphere(m_debugData.Position + m_debugData.Velocity, m_debugData.PointRadius);
            Gizmos.DrawLine(m_debugData.Position, m_debugData.Position + m_debugData.Velocity);
        }

        private void DrawProbes()
        {
            Gizmos.color = Color.green;
            Gizmos.DrawLine(m_debugData.Position, m_debugData.Probe0);
            Gizmos.DrawLine(m_debugData.Position, m_debugData.Probe1);
            Gizmos.DrawLine(m_debugData.Position, m_debugData.Probe2);
            Gizmos.DrawLine(m_debugData.Position, m_debugData.Probe3);
            Gizmos.DrawLine(m_debugData.Position, m_debugData.Probe4);
        }
#endif
#endregion
    }
}