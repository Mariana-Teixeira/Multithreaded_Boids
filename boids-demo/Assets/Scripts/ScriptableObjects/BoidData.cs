using UnityEngine;

[CreateAssetMenu(fileName = "BoidData", menuName = "Data/Boid")]
public class BoidData : ScriptableObject
{
    [Header("Move")]
    [SerializeField] private GameObject m_prefab;
    [SerializeField] private float m_minSpeed;
    [SerializeField] private float m_maxSpeed;
    [SerializeField] private float m_flockAcceleration;
    [SerializeField] private float m_containmentAcceleration;
            
    [Header("Probes")]
    [Tooltip("Probe Length is calculated by multiplying the velocity vector by this multiplier.")]
    [SerializeField] private float m_probeLengthMultiplier;
    [Tooltip("Probe Angle defines the tilt rotation of the four directional probes.")]
    [SerializeField] private float m_probeAngle;

    [Header("Steering")]
    [SerializeField] private float m_separationRadius;
    [Range(-1, 1), SerializeField] private float m_separationDot;
    [SerializeField] private float m_separationWeight;

    [Space, SerializeField] private float m_cohesionRadius;
    [Range(-1, 1), SerializeField] private float m_cohesionDot;
    [SerializeField] private float m_cohesionWeight;

    [Space]
    [SerializeField] private float m_alignmentRadius;
    [Range(-1, 1), SerializeField] private float m_alignmentDot;
    [SerializeField] private float m_alignmentWeight;

    public GameObject Prefab => m_prefab;
    public float MinSpeed => m_minSpeed;
    public float MaxSpeed => m_maxSpeed;
    public float FlockAcceleration => m_flockAcceleration;
    public float ContainmentAcceleration => m_containmentAcceleration;
    public float ProbeLengthMultiplier => m_probeLengthMultiplier;
    public float ProbeAngle => m_probeAngle;
    public float SeparationRadius => m_separationRadius;
    public float SeparationDot => m_separationDot;
    public float SeparationWeight => m_separationWeight;
    public float CohesionRadius => m_cohesionRadius;
    public float CohesionDot => m_cohesionDot;
    public float CohesionWeight => m_cohesionWeight;
    public float AlignmentRadius => m_alignmentRadius;
    public float AlignmentDot => m_alignmentDot;
    public float AlignmentWeight => m_alignmentWeight;
}
