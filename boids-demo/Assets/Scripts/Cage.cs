using UnityEngine;

public class Cage : MonoBehaviour
{
    [SerializeField] private float _cageRadius;
    public float CageRadius => _cageRadius;

    private void OnDrawGizmos()
    {
        Gizmos.color = Color.white;
        Gizmos.DrawWireSphere(transform.position, _cageRadius);
    }
}