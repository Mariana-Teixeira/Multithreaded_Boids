using UnityEngine;
using UnityEngine.UI;

public class DemoInterface : MonoBehaviour
{
    [SerializeField] private WorldData m_worldData;
    
    [SerializeField] private Text m_countText;
    [SerializeField] private Slider m_countSlider;
    [SerializeField] private Dropdown m_modeDropdown;
    
    public void Awake()
    {
        m_modeDropdown.SetValueWithoutNotify((int)m_worldData.OnStartOptimization);
        m_countSlider.SetValueWithoutNotify((int)(m_worldData.OnStartCount / 1000));
        m_countText.text = $"{m_worldData.OnStartCount} Boids";
    }

    public void ChangeCountText(float count)
    {
        m_countText.text = $"{count * 1000} Boids";
    }
}
