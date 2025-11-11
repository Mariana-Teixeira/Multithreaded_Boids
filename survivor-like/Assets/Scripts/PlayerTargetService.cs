using UnityEngine;

public static class PlayerTargetService
{
    public static Vector2 TargetPosition { get; private set; }
    public static void UpdateTargetPosition(Vector2 newPosition)
    {
        TargetPosition = newPosition;
    }
}