using UnityEngine;

[CreateAssetMenu(fileName = "WorldData", menuName = "Data/World")]
public class WorldData : ScriptableObject
{
    [SerializeField] private float m_worldRadius;
    [SerializeField] private float m_spawnRadius;
    [SerializeField] private Optimization m_DefaultOptimization;
    [SerializeField] private int m_defaultCount;

    public float WorldRadius => m_worldRadius;
    public float SpawnRadius => m_spawnRadius;
    public Optimization DefaultOptimization => m_DefaultOptimization;
    public int DefaultCount => m_defaultCount;
}
