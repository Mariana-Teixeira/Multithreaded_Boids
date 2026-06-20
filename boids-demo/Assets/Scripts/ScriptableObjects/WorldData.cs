using UnityEngine;

[CreateAssetMenu(fileName = "WorldData", menuName = "Data/World")]
public class WorldData : ScriptableObject
{
    [SerializeField] private float m_worldRadius;
    [SerializeField] private float m_spawnRadius;
    [SerializeField] private Optimization m_DefaultOptimization;
    [Range(1, 10), SerializeField] private int m_defaultCount;
    [SerializeField] private int m_defaultMultiplier;

    public float WorldRadius => m_worldRadius;
    public float SpawnRadius => m_spawnRadius;
    public Optimization DefaultOptimization => m_DefaultOptimization;
    public int DefaultCount => m_defaultCount;
    public int DefaultMultiplier => m_defaultMultiplier;

    public int GetCount()
    {
        return m_defaultCount * m_defaultMultiplier;
    }
}
