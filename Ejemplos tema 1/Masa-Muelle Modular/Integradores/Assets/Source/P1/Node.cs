using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class Node : MonoBehaviour {

    #region InEditorVariables

    public float Mass;
    public bool Fixed;

    #endregion

    public Vector3 Pos;
    public Vector3 Vel;
    public Vector3 Force;

    public PhysicsManager Manager;

	// Update is called once per frame
	void Update () {
        transform.position = Pos;
	}

    // Use this for initialization
    public void Initialize(PhysicsManager m)
    {
        Manager = m;

        Pos = transform.position;
    }

    // Reset forces
    public void ResetForce()
    {
        Force = Vector3.zero;
    }

    // Compute forces and add
    public void ComputeForce()
    {
        Force += Mass * Manager.Gravity;
    }

}
