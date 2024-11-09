using UnityEngine;

public class AutonomousCarController : MonoBehaviour
{
    public float moveSpeed = 10f;          // Aracın hareket hızı
    public float turnSpeed = 50f;          // Aracın dönüş hızı
    public GameObject[] waypoints;         // Hedef noktaları
    public float stoppingDistance = 2f;    // Hedef noktaya olan minimum mesafe

    private int currentWaypointIndex = 0;  // Şu anki hedef noktanın indeksi

    void Update()
    {
        MoveToWaypoints();
    }

    public void MoveToWaypoints()
    {
        // Hedef noktaya ulaşıldı mı?
        if (Vector3.Distance(transform.position, waypoints[currentWaypointIndex].transform.position) < stoppingDistance)
        {
            currentWaypointIndex = (currentWaypointIndex + 1) % waypoints.Length; // Sonraki hedef noktaya geç
        }

        // Aracı hedef noktaya yönlendir
        Vector3 targetDirection = waypoints[currentWaypointIndex].transform.position - transform.position;
        targetDirection.y = 0f; // Y eksenini ihmal et
        Quaternion targetRotation = Quaternion.LookRotation(targetDirection);
        transform.rotation = Quaternion.Slerp(transform.rotation, targetRotation, turnSpeed * Time.deltaTime);

        // Aracı ileri hareket ettir
        transform.Translate(Vector3.forward * moveSpeed * Time.deltaTime);
    }
}