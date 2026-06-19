using UnityEngine;

[CreateAssetMenu(fileName = "WorldData", menuName = "Data/World")]
public class WorldData : ScriptableObject
{
    [SerializeField] private float m_worldRadius;
    [SerializeField] private float m_spawnRadius;
    [SerializeField] private Optimization m_onStartOptimization;
    [SerializeField] private int m_onStartCount;

    public float WorldRadius => m_worldRadius;
    public float SpawnRadius => m_spawnRadius;
    public Optimization OnStartOptimization => m_onStartOptimization;
    public int OnStartCount => m_onStartCount;
}
