using System;
using UnityEngine;
using UnityEngine.UI;

public class DemoInterface : MonoBehaviour
{
    [SerializeField] private WorldData m_worldData;
    
    [SerializeField] private Text m_countText;
    [SerializeField] private Slider m_countSlider;
    [SerializeField] private Dropdown m_modeDropdown;
    
    private void Start()
    {
        m_countText.text = $"{m_worldData.Count} Boids";
        m_countSlider.value = (int)(m_worldData.Count / 1000);
        m_modeDropdown.value = (int)m_worldData.Configuration;
    }

    public void ChangeCountText(Single count)
    {
        m_countText.text = $"{count * 1000} Boids";
    }
}
