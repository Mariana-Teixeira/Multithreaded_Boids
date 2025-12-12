using UnityEngine;

[CreateAssetMenu(fileName = "VisionConfig", menuName = "Configuration/Vision")]
public class VisionConfiguration : ScriptableObject
{
    [Range(0, 90)] public float VisionAngle = 60.0f; // Half the vision cone angle.
    public float VisionRadius = 30.0f;
}