using System;
using UnityEngine;
using UnityEngine.UI;

[CreateAssetMenu(fileName = "WorldData", menuName = "Data/World")]
public class WorldData : ScriptableObject
{
    public Configuration Configuration;
    public float WorldRadius;
    public float SpawnRadius;
    public int Count;

    public void SetConfiguration(Configuration mode)
    {
        Configuration = mode;
    }

    public void SetConfiguration(int mode)
    {
        Configuration = (Configuration)mode;
    }
    
    public void SetCount(Slider slider)
    {
        Count = (int)slider.value * 1000;
    }

    public void SetCount(int count)
    {
        Count = count;
    }
}
