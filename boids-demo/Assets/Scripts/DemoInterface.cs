using UnityEngine;
using UnityEngine.UI;

public class DemoInterface : MonoBehaviour
{
    [SerializeField] private WorldData m_worldData;
    
    [SerializeField] private Text m_countText;
    [SerializeField] private Slider m_countSlider;
    [SerializeField] private Dropdown m_modeDropdown;
    
    private void Awake()
    {
        m_modeDropdown.SetValueWithoutNotify((int)m_worldData.DefaultOptimization);
        m_countSlider.SetValueWithoutNotify((int)(m_worldData.DefaultCount / m_worldData.DefaultMultiplier));
        m_countText.text = $"{m_worldData.GetCount()} Boids";
    }

    public void ChangeCountText(float count)
    {
        m_countText.text = $"{count * m_worldData.DefaultMultiplier} Boids";
    }
}
