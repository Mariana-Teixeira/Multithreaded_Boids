using UnityEngine;

[CreateAssetMenu(fileName = "WorldData", menuName = "Data/World")]
public class WorldData : ScriptableObject
{
    [SerializeField] private float m_defaultWorldRadius;
    [Range(1, 10), SerializeField] private int m_defaultCount;
    [SerializeField] private int m_defaultMultiplier;

    public float DefaultWorldRadius => m_defaultWorldRadius;
    public int DefaultCount => m_defaultCount;
    public int DefaultMultiplier => m_defaultMultiplier;

    public int GetMinCount => 1;
    public int GetMaxCount => 10;
    
    public float GetMaxSpawnRadius()
    {
        return m_defaultWorldRadius * 0.95f;
    }

    public int GetCount()
    {
        return m_defaultCount * m_defaultMultiplier;
    }
}
