using UnityEngine;
using UnityEngine.InputSystem;

public class PlayerMovement : MonoBehaviour
{
    [Header("Movement")]
    [SerializeField] private float _moveSpeed;
    
    private InputAction _moveAction;
    private Rigidbody2D _rigidbody;
    private Vector2 _moveDirection;

    private void Awake()
    {
        _rigidbody = GetComponent<Rigidbody2D>();
        _moveAction = InputSystem.actions.FindAction("Player/Move");
    }

    private void FixedUpdate()
    {
        _moveDirection = _moveAction.ReadValue<Vector2>();
        _rigidbody.linearVelocity = _moveDirection * _moveSpeed;
        
        PlayerTargetService.UpdateTargetPosition(_rigidbody.position);
    }
}