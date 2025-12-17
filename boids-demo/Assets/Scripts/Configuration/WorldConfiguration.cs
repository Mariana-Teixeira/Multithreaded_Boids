using UnityEngine;

[CreateAssetMenu(fileName = "WorldConfig", menuName = "Configuration/World")]
public class WorldConfiguration : ScriptableObject
{
    public Vector3 CageCenter = Vector3.zero;
    public float CageRadius = 20.0f;
}