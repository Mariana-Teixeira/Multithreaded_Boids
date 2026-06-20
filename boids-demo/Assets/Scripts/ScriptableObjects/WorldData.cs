using UnityEngine;

[CreateAssetMenu(fileName = "WorldData", menuName = "Data/World")]
public class WorldData : ScriptableObject
{
    [SerializeField] private float m_defaultWorldRadius;
    [SerializeField] private float m_defaultSpawnRadius;
    [Range(1, 10), SerializeField] private int m_defaultCount;
    [SerializeField] private int m_defaultMultiplier;

    public float DefaultWorldRadius => m_defaultWorldRadius;
    public float DefaultSpawnRadius => m_defaultSpawnRadius;
    public int DefaultCount => m_defaultCount;
    public int DefaultMultiplier => m_defaultMultiplier;

    public int GetCount()
    {
        return m_defaultCount * m_defaultMultiplier;
    }
}
