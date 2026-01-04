using System;
using UnityEngine;

namespace Boids
{
    [Serializable]
    public struct Settings
    {
        public Vector3 GridCenter;
        public float GridRadius;
        public float CellRadius;
        public GameObject BoidPrefab;
        public int BoidCount;
        public float VisionAngle;
        public float VisionRadius;
        public float MinSpeed;
        public float MaxSpeed;
        public float RotationSpeed;
        public float SpringWeight;
        public float AlignmentWeight;
        public float SeparationWeight;
        public float CohesionWeight;
        
        [HideInInspector] public float VisionThreshold;
    }
}