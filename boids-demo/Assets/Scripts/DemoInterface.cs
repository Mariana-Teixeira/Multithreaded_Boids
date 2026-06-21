using UnityEngine;
using UnityEngine.UI;

public class DemoInterface : MonoBehaviour
{
    [SerializeField] private BoidData m_boidData;
    [SerializeField] private WorldData m_worldData;
    
    [SerializeField] private Text m_countText;
    [SerializeField] private Slider m_countSlider;
    
    private void Awake()
    {
        m_countSlider.minValue = m_worldData.GetMinCount;
        m_countSlider.maxValue = m_worldData.GetMaxCount;
        m_countSlider.SetValueWithoutNotify(m_worldData.DefaultCount);
        m_countText.text = $"{m_worldData.GetCount()} Boids";
    }

    public void ChangeCountText(float count)
    {
        m_countText.text = $"{count * m_worldData.DefaultMultiplier} Boids";
    }
}
